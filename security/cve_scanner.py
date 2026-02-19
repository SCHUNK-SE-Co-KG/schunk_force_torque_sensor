#!/usr/bin/env python3
"""
CVE Scanner for the schunk_force_torque_sensor repository.

Scans three ecosystems for known security vulnerabilities:
  1. Python pip packages  - from setup.py install_requires (PyPI)
  2. Rust Crates          - from Cargo.lock (crates.io)
  3. ROS 2 dependencies   - from package.xml (GitHub Advisory DB)

Data sources:
  - Google OSV API  (https://osv.dev)
  - GitHub Advisory Database  (REST API + web scraping)

Results are stored as Markdown report and JSON under
security/reports/.

Usage:
    python security/cve_scanner.py                # Scan + Report
    python security/cve_scanner.py --json-only    # JSON output only

Requirements:
    Python 3.9+, requests (pip install requests)

Author:  SCHUNK Security Tooling
License: GPL-3.0
"""

from __future__ import annotations

import argparse
import ast
import json
import os
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

# ── Force UTF-8 on stdout (Windows cp1252 crashes on emojis) ──
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

try:
    import requests
except ImportError:
    sys.exit("Error: 'requests' is not installed.\n" "  pip install requests")

# ── Constants ────────────────────────────────────────────────────────
ROOT_DIR = Path(__file__).resolve().parent.parent
REPORT_DIR = ROOT_DIR / "security" / "reports"
REPO_NAME = "SCHUNK-SE-Co-KG/schunk_force_torque_sensor"

OSV_QUERY_URL = "https://api.osv.dev/v1/query"
OSV_BATCH_URL = "https://api.osv.dev/v1/querybatch"
OSV_VULN_URL = "https://api.osv.dev/v1/vulns"  # + /{id}

CVSS_HIGH_THRESHOLD = 7.0  # Alert threshold

# Severity label -> estimated CVSS base score (last-resort fallback)
SEVERITY_LABEL_SCORES: dict[str, float] = {
    "CRITICAL": 9.0,
    "HIGH": 7.5,
    "MODERATE": 5.0,
    "MEDIUM": 5.0,
    "LOW": 2.5,
}

# ── Internal packages (skip during scan) ────────────────────────────
INTERNAL_PACKAGES = {
    "schunk_fts_driver",
    "schunk_fts_library",
    "schunk_fts_interfaces",
    "schunk_fts_dummy",
}

# Build-tool packages (no security risk)
BUILD_TOOL_PACKAGES = {
    "ament_cmake",
    "ament_cmake_pytest",
    "ament_cmake_copyright",
    "ament_lint_auto",
    "ament_lint_common",
    "ament_copyright",
    "rosidl_default_generators",
    "rosidl_default_runtime",
    "launch_pytest",
    "python3-pytest",
}

# ROS 2 packages with known upstream sources (scannable via GitHub Advisory)
ROS_UPSTREAM_REPOS: dict[str, str] = {
    "rclpy": "ros2/rclpy",
    "launch": "ros2/launch",
    "launch_ros": "ros2/launch_ros",
    "geometry_msgs": "ros2/common_interfaces",
    "std_srvs": "ros2/common_interfaces",
    "sensor_msgs": "ros2/common_interfaces",
    "diagnostic_msgs": "ros2/common_interfaces",
    "std_msgs": "ros2/common_interfaces",
    "example_interfaces": "ros2/example_interfaces",
}

# Python packages that are build/dev-tools only (informational, but scannable)
PYTHON_DEV_PACKAGES = {
    "setuptools",
    "black",
    "click",
    "exceptiongroup",
    "iniconfig",
    "mypy_extensions",
    "packaging",
    "pathspec",
    "platformdirs",
    "pluggy",
    "pytest",
    "pytest-repeat",
    "tomli",
    "typing_extensions",
    "lark",
    "coverage",
    "flake8",
    "mypy",
    "pre-commit",
}


# ── Data structures ───────────────────────────────────────────────────
@dataclass
class Dependency:
    """A detected dependency."""

    name: str
    version: str
    ecosystem: str  # "PyPI", "crates.io", "ROS", "GitHub"
    source: str  # File where it was found
    upstream_repo: str = ""  # GitHub repo (if known)


# ── CVSS Score extraction ────────────────────────────────────────────
def _fetch_ghsa_cvss_score(ghsa_id: str) -> Optional[float]:
    """Fetch the official numeric CVSS score for a GitHub Advisory.

    Strategy (priority):
    1. Scrape GitHub Advisory web page - always shows the latest score
       (CVSS 3.x or 4.0), pattern: ">X.X<" before "/ 10"
    2. GitHub REST API (/advisories/{id}) - fallback, may return outdated
       CVSS 3.1 scores when advisory was updated to CVSS 4.0
    """
    # 1. Scrape advisory web page (primary source - always up-to-date)
    page_url = f"https://github.com/advisories/{ghsa_id}"
    try:
        resp = requests.get(
            page_url,
            timeout=10,
            headers={
                "Accept": "text/html",
                "User-Agent": "SCHUNK-CVE-Scanner/1.0",
            },
        )
        if resp.status_code == 200:
            idx = resp.text.find("/ 10")
            if idx > 0:
                window = resp.text[max(0, idx - 3000) : idx]
                numbers = re.findall(r">(\d+\.\d+)<", window)
                if numbers:
                    return float(numbers[-1])
    except Exception:
        pass

    # 2. REST API as fallback
    url = f"https://api.github.com/advisories/{ghsa_id}"
    try:
        resp = requests.get(
            url,
            timeout=10,
            headers={
                "Accept": "application/vnd.github+json",
                "X-GitHub-Api-Version": "2022-11-28",
            },
        )
        if resp.status_code == 200:
            data = resp.json()
            for field_name in ("cvss_v4", "cvss"):
                cvss_data = data.get(field_name)
                if cvss_data and isinstance(cvss_data, dict):
                    score = cvss_data.get("score")
                    if isinstance(score, (int, float)) and score > 0:
                        return float(score)
    except Exception:
        pass

    return None


@dataclass
class Vulnerability:
    """A detected security vulnerability."""

    vuln_id: str
    summary: str
    severity: str
    cvss_score: float
    affected_package: str
    affected_versions: str
    fixed_version: str
    references: list[str] = field(default_factory=list)
    aliases: list[str] = field(default_factory=list)


@dataclass
class ScanResult:
    """Overall result of a scan run."""

    timestamp: str
    repository: str
    dependencies_scanned: int
    vulnerabilities_found: int
    high_severity_count: int = 0
    dependencies: list[Dependency] = field(default_factory=list)
    vulnerabilities: list[Vulnerability] = field(default_factory=list)


# ═══════════════════════════════════════════════════════════════
# 1) Python pip dependencies from setup.py (AST-based parsing)
# ═══════════════════════════════════════════════════════════════
def _extract_string_list(node: ast.expr) -> list[str]:
    """Extract string values from an AST node (List, Tuple, or single string)."""
    strings: list[str] = []
    if isinstance(node, (ast.List, ast.Tuple)):
        for elt in node.elts:
            if isinstance(elt, ast.Constant) and isinstance(elt.value, str):
                strings.append(elt.value)
    elif isinstance(node, ast.Constant) and isinstance(node.value, str):
        strings.append(node.value)
    return strings


def _parse_requirement_string(req: str) -> tuple[str, str]:
    """Parse a PEP 508 requirement string into (name, version).

    Handles formats like:
      "numpy==2.2.6"
      "numpy>=2.2.6"
      "numpy>=2.2.6, <3"
      "numpy"             (no version)
      "requests[security]>=2.31"

    Returns (name, first_version) or (name, "") if no version specified.
    """
    req = req.strip()
    if not req:
        return ("", "")

    # Split on version operators: ==, >=, <=, ~=, !=, <, >
    match = re.match(r"^([a-zA-Z0-9_.-]+(?:\[[^\]]*\])?)\s*(.*)", req)
    if not match:
        return ("", "")

    name_part = match.group(1)
    version_part = match.group(2).strip()

    # Remove extras like [security]
    name = re.sub(r"\[.*?\]", "", name_part).strip()

    if not version_part:
        return (name, "")

    # Extract the first version number
    ver_match = re.match(r"(?:==|>=|<=|~=|!=|<|>)\s*([^\s,;]+)", version_part)
    if ver_match:
        return (name, ver_match.group(1))

    return (name, "")


def _parse_setup_py_install_requires(content: str) -> list[tuple[str, str]]:
    """Parse install_requires from setup.py using Python AST.

    Returns list of (package_name, version_string) tuples.
    More robust than regex: handles comments, parentheses, logic in
    install_requires, and all PEP 508 version specifiers.
    """
    try:
        tree = ast.parse(content)
    except SyntaxError:
        return []

    requirements: list[tuple[str, str]] = []

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue

        # Find setup() or setuptools.setup() call
        func = node.func
        func_name = ""
        if isinstance(func, ast.Name):
            func_name = func.id
        elif isinstance(func, ast.Attribute):
            func_name = func.attr

        if func_name != "setup":
            continue

        # Find install_requires keyword argument
        for kw in node.keywords:
            if kw.arg != "install_requires":
                continue

            req_strings = _extract_string_list(kw.value)
            for req_str in req_strings:
                name, version = _parse_requirement_string(req_str)
                if name:
                    requirements.append((name, version))

    return requirements


def extract_setup_py_deps(root_dir: Path) -> list[Dependency]:
    """Extract Python packages from all setup.py install_requires using AST.

    Uses Python's ast module instead of regex for robust parsing.
    Keys dedup on (normalized_name, version) so that different versions
    of the same package across multiple setup.py files are all scanned.
    """
    deps: list[Dependency] = []
    seen: set[str] = set()

    # Normalize INTERNAL_PACKAGES for comparison
    internal_norm = {p.lower().replace("-", "_") for p in INTERNAL_PACKAGES}

    for setup_py in root_dir.rglob("setup.py"):
        try:
            content = setup_py.read_text(encoding="utf-8")
        except Exception:
            continue

        source = str(setup_py.relative_to(root_dir))
        requirements = _parse_setup_py_install_requires(content)

        for name, version in requirements:
            norm_name = name.lower().replace("-", "_")

            # Dedup key includes version so different versions are kept
            dedup_key = f"{norm_name}:{version}"
            if dedup_key in seen:
                continue
            seen.add(dedup_key)

            # Skip internal packages
            if norm_name in internal_norm:
                continue

            deps.append(
                Dependency(
                    name=name,
                    version=version,
                    ecosystem="PyPI",
                    source=source,
                )
            )

    return deps


# ═══════════════════════════════════════════════════════════════
# 2) Rust crate dependencies from Cargo.lock
# ═══════════════════════════════════════════════════════════════
def extract_cargo_lock_deps(root_dir: Path) -> list[Dependency]:
    """Extract packages + versions from all Cargo.lock files.

    Keys dedup on (name, version) so that multiple versions of the
    same crate are all scanned individually.
    """
    deps: list[Dependency] = []
    seen: set[str] = set()

    # Cargo.lock format:
    # [[package]]
    # name = "tokio"
    # version = "1.45.0"
    name_re = re.compile(r'^name\s*=\s*"([^"]+)"', re.MULTILINE)
    version_re = re.compile(r'^version\s*=\s*"([^"]+)"', re.MULTILINE)

    for cargo_lock in root_dir.rglob("Cargo.lock"):
        try:
            content = cargo_lock.read_text(encoding="utf-8")
        except Exception:
            continue

        source = str(cargo_lock.relative_to(root_dir))

        # Each [[package]] block contains name + version
        blocks = content.split("[[package]]")
        for block in blocks[1:]:  # First part is header
            name_match = name_re.search(block)
            ver_match = version_re.search(block)
            if not name_match or not ver_match:
                continue

            name = name_match.group(1)
            version = ver_match.group(1)

            # Dedup key includes version (Cargo.lock can have multiple
            # versions of the same crate)
            dedup_key = f"{name}:{version}"
            if dedup_key in seen:
                continue
            seen.add(dedup_key)

            # Skip internal packages (uses constant at top of file)
            if name in INTERNAL_PACKAGES:
                continue

            deps.append(
                Dependency(
                    name=name,
                    version=version,
                    ecosystem="crates.io",
                    source=source,
                )
            )

    return deps


# ═══════════════════════════════════════════════════════════════
# 3) ROS 2 dependencies from package.xml
# ═══════════════════════════════════════════════════════════════
def extract_package_xml_deps(root_dir: Path) -> list[Dependency]:
    """Extract dependencies from all package.xml files."""
    deps: list[Dependency] = []
    seen: set[str] = set()

    dep_tags = [
        "depend",
        "build_depend",
        "buildtool_depend",
        "exec_depend",
        "run_depend",
        "test_depend",
    ]

    for pkg_xml in root_dir.rglob("package.xml"):
        try:
            tree = ET.parse(pkg_xml)
            root = tree.getroot()
        except ET.ParseError as e:
            print(f"  WARNING  XML-Fehler in {pkg_xml}: {e}")
            continue

        source = str(pkg_xml.relative_to(root_dir))

        for tag in dep_tags:
            for elem in root.findall(tag):
                name = elem.text.strip() if elem.text else ""
                if not name:
                    continue

                # Skip internal and build-tool packages
                if name in INTERNAL_PACKAGES or name in BUILD_TOOL_PACKAGES:
                    continue

                if name in seen:
                    continue
                seen.add(name)

                if name in ROS_UPSTREAM_REPOS:
                    deps.append(
                        Dependency(
                            name=name,
                            version="",
                            ecosystem="ROS",
                            source=source,
                            upstream_repo=ROS_UPSTREAM_REPOS[name],
                        )
                    )
                else:
                    # Generic ROS package without known mapping
                    deps.append(
                        Dependency(
                            name=name,
                            version="",
                            ecosystem="ROS",
                            source=source,
                        )
                    )

    return deps


# ═══════════════════════════════════════════════════════════════
# Dedup + Merge
# ═══════════════════════════════════════════════════════════════
def deduplicate_deps(deps: list[Dependency]) -> list[Dependency]:
    """Remove duplicate dependencies (same name + ecosystem + version)."""
    seen: set[str] = set()
    unique: list[Dependency] = []
    for d in deps:
        key = f"{d.ecosystem}:{d.name}:{d.version}"
        if key not in seen:
            seen.add(key)
            unique.append(d)
    return unique


# ── GitHub Advisory scan for ROS upstream repos ──────────────────────
def scan_github_advisories(dep: Dependency) -> list[dict]:
    """Scan GitHub Advisory Database for a specific repository.

    Queries multiple ecosystems since ROS packages may have advisories
    filed under different ecosystems (pip, Go, etc.).
    Falls back to the repository's own security advisories endpoint.
    """
    if not dep.upstream_repo:
        return []

    all_advisories: list[dict] = []
    seen_ids: set[str] = set()

    # Try global advisories endpoint with different ecosystems
    # (ROS packages are not a recognized ecosystem in GitHub Advisory DB)
    url = "https://api.github.com/advisories"
    for ecosystem in ("pip", "go", "rust", "npm"):
        try:
            resp = requests.get(
                url,
                timeout=15,
                params={
                    "ecosystem": ecosystem,
                    "affects": dep.upstream_repo,
                },
                headers={
                    "Accept": "application/vnd.github+json",
                    "X-GitHub-Api-Version": "2022-11-28",
                },
            )
            if resp.status_code == 200:
                for adv in resp.json():
                    adv_id = adv.get("ghsa_id", "")
                    if adv_id and adv_id not in seen_ids:
                        seen_ids.add(adv_id)
                        all_advisories.append(adv)
        except Exception:
            pass

    # Also check repository-specific security advisories
    repo_url = f"https://api.github.com/repos/{dep.upstream_repo}/security-advisories"
    try:
        resp = requests.get(
            repo_url,
            timeout=15,
            headers={
                "Accept": "application/vnd.github+json",
                "X-GitHub-Api-Version": "2022-11-28",
            },
        )
        if resp.status_code == 200:
            for adv in resp.json():
                adv_id = adv.get("ghsa_id", "")
                if adv_id and adv_id not in seen_ids:
                    seen_ids.add(adv_id)
                    all_advisories.append(adv)
    except Exception:
        pass

    return all_advisories


# ── OSV-API-Abfragen ────────────────────────────────────────────────
def _build_osv_query(dep: Dependency) -> dict:
    """Baut eine einzelne OSV-Query für eine Abhängigkeit."""
    q: dict = {"package": {"name": dep.name, "ecosystem": dep.ecosystem}}
    if dep.version:
        q["version"] = dep.version
    return q


def query_osv_batch(deps: list[Dependency]) -> list[list[dict]]:
    """Sendet einen Batch-Query an die OSV-API."""
    if not deps:
        return []

    # Only query deps with a known OSV ecosystem
    osv_ecosystems = {"PyPI", "Debian", "npm", "crates.io", "Go", "Maven"}

    queries = []
    query_indices = []
    for i, dep in enumerate(deps):
        if dep.ecosystem in osv_ecosystems:
            queries.append(_build_osv_query(dep))
            query_indices.append(i)

    results: list[list[dict]] = [[] for _ in deps]

    if queries:
        try:
            resp = requests.post(
                OSV_BATCH_URL,
                json={"queries": queries},
                timeout=30,
            )
            resp.raise_for_status()
            batch_results = resp.json().get("results", [])
            for j, res in enumerate(batch_results):
                orig_idx = query_indices[j]
                results[orig_idx] = res.get("vulns", [])
        except requests.RequestException as exc:
            print(f"  WARNING  OSV-API Batch-Fehler: {exc}")

    return results


def fetch_vuln_details(vuln_id: str) -> Optional[dict]:
    """Fetch full vulnerability details from the OSV API."""
    try:
        resp = requests.get(f"{OSV_VULN_URL}/{vuln_id}", timeout=15)
        resp.raise_for_status()
        return resp.json()
    except requests.RequestException:
        return None


def enrich_vulnerabilities(vulns_per_dep: list[list[dict]]) -> list[list[dict]]:
    """Enrich batch results with full details."""
    enriched: list[list[dict]] = []
    for dep_vulns in vulns_per_dep:
        enriched_list: list[dict] = []
        for v in dep_vulns:
            vuln_id = v.get("id", "")
            if not v.get("summary") and not v.get("details"):
                full = fetch_vuln_details(vuln_id)
                if full:
                    enriched_list.append(full)
                    continue
            enriched_list.append(v)
        enriched.append(enriched_list)
    return enriched


# ── Vulnerability parsing ──────────────────────────────────────────
def _extract_severity(vuln: dict) -> str:
    """Extract severity level from an OSV vulnerability."""
    severity_entries = vuln.get("severity", [])
    for s in severity_entries:
        score_str = s.get("score", "")
        if score_str:
            return score_str
    db_spec = vuln.get("database_specific", {})
    severity = db_spec.get("severity", "")
    if severity:
        return severity
    return "UNKNOWN"


def _compute_cvss_score(vuln: dict, severity_str: str) -> float:
    """Fetch the official CVSS score from the GitHub Advisory API."""
    vuln_id = vuln.get("id", "")

    # 1. Determine GHSA ID
    ghsa_id = vuln_id if vuln_id.startswith("GHSA-") else ""
    if not ghsa_id:
        for alias in vuln.get("aliases", []):
            if alias.startswith("GHSA-"):
                ghsa_id = alias
                break

    if ghsa_id:
        api_score = _fetch_ghsa_cvss_score(ghsa_id)
        if api_score is not None:
            return api_score

    # 2. Fallback: Severity label
    db_severity = vuln.get("database_specific", {}).get("severity", "").upper()
    if db_severity in SEVERITY_LABEL_SCORES:
        return SEVERITY_LABEL_SCORES[db_severity]

    upper = severity_str.upper()
    for label in SEVERITY_LABEL_SCORES:
        if label in upper:
            return SEVERITY_LABEL_SCORES[label]

    return 0.0


def _extract_fixed_version(affected: dict) -> str:
    """Determine the first fixed version from affected ranges."""
    for rng in affected.get("ranges", []):
        for event in rng.get("events", []):
            if "fixed" in event:
                return event["fixed"]
    return "–"


def parse_vulnerabilities(dep: Dependency, vulns: list[dict]) -> list[Vulnerability]:
    """Convert raw OSV entries to Vulnerability objects."""
    results: list[Vulnerability] = []
    for v in vulns:
        vuln_id = v.get("id", "?")
        summary = v.get("summary", v.get("details", "Keine Beschreibung"))
        severity = _extract_severity(v)
        aliases = v.get("aliases", [])

        affected_versions = ""
        fixed_version = "–"
        for aff in v.get("affected", []):
            pkg = aff.get("package", {})
            if pkg.get("name") == dep.name:
                affected_versions = ", ".join(aff.get("versions", [])[:10])
                if len(aff.get("versions", [])) > 10:
                    affected_versions += " …"
                fixed_version = _extract_fixed_version(aff)
                break

        refs = [r.get("url", "") for r in v.get("references", []) if r.get("url")]
        cvss_score = _compute_cvss_score(v, severity)

        results.append(
            Vulnerability(
                vuln_id=vuln_id,
                summary=summary[:300],
                severity=severity,
                cvss_score=cvss_score,
                affected_package=f"{dep.ecosystem}:{dep.name}"
                + (f"@{dep.version}" if dep.version else ""),
                affected_versions=affected_versions,
                fixed_version=fixed_version,
                references=refs[:5],
                aliases=aliases,
            )
        )
    return results


def parse_github_advisory(dep: Dependency, advisory: dict) -> Optional[Vulnerability]:
    """Convert a GitHub Advisory into a Vulnerability object."""
    ghsa_id = advisory.get("ghsa_id", "")
    if not ghsa_id:
        return None

    summary = advisory.get("summary", "Keine Beschreibung")
    severity = advisory.get("severity", "UNKNOWN")
    cve_id = advisory.get("cve_id", "")

    cvss_score = 0.0
    score = _fetch_ghsa_cvss_score(ghsa_id)
    if score is not None:
        cvss_score = score
    elif severity.upper() in SEVERITY_LABEL_SCORES:
        cvss_score = SEVERITY_LABEL_SCORES[severity.upper()]

    refs = []
    if advisory.get("html_url"):
        refs.append(advisory["html_url"])

    aliases = [ghsa_id]
    if cve_id:
        aliases.append(cve_id)

    return Vulnerability(
        vuln_id=ghsa_id,
        summary=summary[:300],
        severity=severity,
        cvss_score=cvss_score,
        affected_package=f"ROS:{dep.name}"
        + (f" ({dep.upstream_repo})" if dep.upstream_repo else ""),
        affected_versions="",
        fixed_version="–",
        references=refs[:5],
        aliases=aliases,
    )


# ── Report generation ──────────────────────────────────────────────
def generate_markdown_report(result: ScanResult) -> str:
    """Generate a Markdown report from the scan result."""
    lines: list[str] = []
    lines.append("# CVE-Scan Report – schunk_force_torque_sensor")
    lines.append("")
    lines.append(f"**Scan-Zeitpunkt:** {result.timestamp}")
    lines.append(f"**Repository:** {result.repository}")
    lines.append(f"**Abhängigkeiten geprüft:** {result.dependencies_scanned}")
    lines.append(f"**Schwachstellen gefunden:** {result.vulnerabilities_found}")
    lines.append("")

    if result.vulnerabilities_found == 0:
        lines.append("> Keine bekannten Schwachstellen gefunden.")
    else:
        lines.append(f"> {result.vulnerabilities_found} Schwachstelle(n) gefunden!")
    lines.append("")

    # Aufschlüsselung nach Ökosystem
    pypi_deps = [d for d in result.dependencies if d.ecosystem == "PyPI"]
    crate_deps = [d for d in result.dependencies if d.ecosystem == "crates.io"]
    ros_deps = [d for d in result.dependencies if d.ecosystem == "ROS"]

    lines.append("## Zusammenfassung nach Ökosystem")
    lines.append("")
    lines.append("| Ökosystem | Abhängigkeiten | Schwachstellen |")
    lines.append("|-----------|---------------|----------------|")

    pypi_vulns = [
        v for v in result.vulnerabilities if v.affected_package.startswith("PyPI:")
    ]
    crate_vulns = [
        v for v in result.vulnerabilities if v.affected_package.startswith("crates.io:")
    ]
    ros_vulns = [
        v for v in result.vulnerabilities if v.affected_package.startswith("ROS:")
    ]

    lines.append(f"| PyPI (Python) | {len(pypi_deps)} | {len(pypi_vulns)} |")
    lines.append(f"| crates.io (Rust) | {len(crate_deps)} | {len(crate_vulns)} |")
    lines.append(f"| ROS 2 | {len(ros_deps)} | {len(ros_vulns)} |")
    lines.append(
        f"| **Gesamt** | **{result.dependencies_scanned}**"
        f" | **{result.vulnerabilities_found}** |"
    )
    lines.append("")

    # Geprüfte Abhängigkeiten
    lines.append("## Geprüfte Abhängigkeiten")
    lines.append("")

    if pypi_deps:
        lines.append("### Python (PyPI)")
        lines.append("")
        lines.append("| Paket | Version | Quelle |")
        lines.append("|-------|---------|--------|")
        for d in pypi_deps:
            lines.append(f"| {d.name} | {d.version or '–'} | {d.source} |")
        lines.append("")

    if crate_deps:
        lines.append("### Rust (crates.io)")
        lines.append("")
        lines.append("| Crate | Version | Quelle |")
        lines.append("|-------|---------|--------|")
        for d in crate_deps:
            lines.append(f"| {d.name} | {d.version or '–'} | {d.source} |")
        lines.append("")

    if ros_deps:
        lines.append("### ROS 2")
        lines.append("")
        lines.append("| Paket | Ökosystem | Quelle | Upstream |")
        lines.append("|-------|-----------|--------|----------|")
        for d in ros_deps:
            upstream = (
                f"[{d.upstream_repo}](https://github.com/{d.upstream_repo})"
                if d.upstream_repo
                else "–"
            )
            lines.append(f"| {d.name} | {d.ecosystem} | {d.source} | {upstream} |")
        lines.append("")

    # Schwachstellen
    if result.vulnerabilities:
        lines.append("## Gefundene Schwachstellen")
        lines.append("")
        for v in result.vulnerabilities:
            cve_aliases = [a for a in v.aliases if a.startswith("CVE-")]
            cve_str = ", ".join(cve_aliases) if cve_aliases else "–"
            lines.append(f"### {v.vuln_id}")
            lines.append("")
            lines.append(f"- **Paket:** {v.affected_package}")
            score_display = f"{v.cvss_score:.1f}" if v.cvss_score > 0 else "–"
            high_marker = " (KRITISCH)" if v.cvss_score >= CVSS_HIGH_THRESHOLD else ""
            lines.append(f"- **CVSS-Score:** {score_display}{high_marker}")
            lines.append(f"- **Schweregrad:** {v.severity}")
            lines.append(f"- **CVE:** {cve_str}")
            lines.append(f"- **Beschreibung:** {v.summary}")
            lines.append(f"- **Fix-Version:** {v.fixed_version}")
            if v.references:
                lines.append("- **Referenzen:**")
                for ref in v.references:
                    lines.append(f"  - {ref}")
            lines.append("")

    lines.append("---")
    lines.append(
        "*Automatisch generiert von `security/cve_scanner.py` "
        "via [OSV.dev](https://osv.dev) und GitHub Advisory Database.*"
    )
    lines.append("")
    return "\n".join(lines)


def generate_alert_body(result: ScanResult) -> str:
    """Generate the body for the high-severity alert."""
    high = [v for v in result.vulnerabilities if v.cvss_score >= CVSS_HIGH_THRESHOLD]
    lines: list[str] = []
    lines.append("CVE-Alert: Kritische Schwachstellen in schunk_force_torque_sensor")
    lines.append("=" * 68)
    lines.append("")
    lines.append(f"Scan-Zeitpunkt: {result.timestamp}")
    lines.append(f"Repository:     {result.repository}")
    lines.append(
        f"Schwachstellen: {result.vulnerabilities_found} gesamt, "
        f"{len(high)} mit CVSS >= {CVSS_HIGH_THRESHOLD}"
    )
    lines.append("")
    lines.append("-" * 68)

    for v in high:
        cve_aliases = [a for a in v.aliases if a.startswith("CVE-")]
        cve_str = ", ".join(cve_aliases) if cve_aliases else "–"
        lines.append("")
        lines.append(f"  {v.vuln_id}  (CVSS {v.cvss_score:.1f})")
        lines.append(f"  Paket:       {v.affected_package}")
        lines.append(f"  CVE:         {cve_str}")
        lines.append(f"  Fix-Version: {v.fixed_version}")
        lines.append(f"  {v.summary[:200]}")
        if v.references:
            lines.append(f"  -> {v.references[0]}")

    lines.append("")
    lines.append("-" * 68)
    lines.append("Vollstaendiger Report: security/reports/cve_report.md")
    lines.append(
        f"https://github.com/{REPO_NAME}/blob/main/security/reports/cve_report.md"
    )
    lines.append("")
    return "\n".join(lines)


def write_alert_output(result: ScanResult, report_dir: Path) -> None:
    """Write alert files for GitHub Actions."""
    report_dir.mkdir(parents=True, exist_ok=True)
    alert_flag_path = report_dir / "alert_high_severity.txt"

    if result.high_severity_count > 0:
        alert_body = generate_alert_body(result)
        alert_flag_path.write_text(alert_body, encoding="utf-8")
        print(f"  Alert-Datei: {alert_flag_path.relative_to(ROOT_DIR)}")

        gh_output = os.environ.get("GITHUB_OUTPUT")
        if gh_output:
            with open(gh_output, "a") as f:
                f.write("high_severity=true\n")
                f.write(f"high_count={result.high_severity_count}\n")
                f.write(
                    f"alert_subject=CVE-Alert: {result.high_severity_count} "
                    "kritische Schwachstelle(n) in schunk_force_torque_sensor\n"
                )
    else:
        if alert_flag_path.exists():
            alert_flag_path.unlink()
        gh_output = os.environ.get("GITHUB_OUTPUT")
        if gh_output:
            with open(gh_output, "a") as f:
                f.write("high_severity=false\n")


def save_reports(result: ScanResult, report_dir: Path) -> tuple[Path, Path]:
    """Save JSON and Markdown reports."""
    report_dir.mkdir(parents=True, exist_ok=True)

    json_path = report_dir / "cve_report.json"
    md_path = report_dir / "cve_report.md"

    json_path.write_text(
        json.dumps(asdict(result), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(generate_markdown_report(result), encoding="utf-8")

    return json_path, md_path


# ── Main logic ───────────────────────────────────────────────────────
def run_scan() -> ScanResult:
    """Execute the full CVE scan."""
    print("CVE-Scanner gestartet ...")
    print(f"   Repository-Root: {ROOT_DIR}")
    print()

    # 1. Abhängigkeiten aus allen Quellen sammeln
    print("== Python pip-Abhaengigkeiten (setup.py) ==")
    pip_deps = extract_setup_py_deps(ROOT_DIR)
    for d in pip_deps:
        print(f"   * {d.name}=={d.version}  ({d.source})")

    print()
    print("== Rust Crate-Abhaengigkeiten (Cargo.lock) ==")
    cargo_deps = extract_cargo_lock_deps(ROOT_DIR)
    for d in cargo_deps:
        print(f"   * {d.name}@{d.version}  ({d.source})")

    print()
    print("== ROS 2 Abhaengigkeiten (package.xml) ==")
    ros_deps = extract_package_xml_deps(ROOT_DIR)
    for d in ros_deps:
        upstream = f" -> {d.upstream_repo}" if d.upstream_repo else ""
        print(f"   * {d.name}  ({d.ecosystem}){upstream}")

    all_deps = deduplicate_deps(pip_deps + cargo_deps + ros_deps)
    print()
    print(f"Gesamt: {len(all_deps)} Abhaengigkeit(en) erkannt")
    print(f"   PyPI:      {len([d for d in all_deps if d.ecosystem == 'PyPI'])}")
    print(f"   crates.io: {len([d for d in all_deps if d.ecosystem == 'crates.io'])}")
    print(f"   ROS 2:     {len([d for d in all_deps if d.ecosystem == 'ROS'])}")
    print()

    # 2. OSV-API Batch-Abfrage (PyPI + crates.io)
    print("Frage OSV-API ab (PyPI + crates.io) ...")
    batch_results = query_osv_batch(all_deps)
    batch_results = enrich_vulnerabilities(batch_results)

    # 3. GitHub Advisory Scan für ROS-Pakete mit bekanntem Upstream
    print("Pruefe GitHub Advisory Database fuer ROS-Upstream-Repos ...")
    all_vulns: list[Vulnerability] = []

    for dep, osv_vulns in zip(all_deps, batch_results):
        # OSV-Ergebnisse verarbeiten
        parsed = parse_vulnerabilities(dep, osv_vulns)
        if parsed:
            print(f"   !! {dep.name}: {len(parsed)} Schwachstelle(n) (OSV)")
        all_vulns.extend(parsed)

        # GitHub Advisories für ROS-Upstream-Repos
        if dep.upstream_repo:
            gh_advisories = scan_github_advisories(dep)
            seen_ids = {v.vuln_id for v in all_vulns}
            for adv in gh_advisories:
                vuln = parse_github_advisory(dep, adv)
                if vuln and vuln.vuln_id not in seen_ids:
                    all_vulns.append(vuln)
                    seen_ids.add(vuln.vuln_id)
                    print(f"   !! {dep.name}: {vuln.vuln_id} (GitHub Advisory)")

    if not all_vulns:
        print("   OK Keine Schwachstellen gefunden.")

    high_sev = [v for v in all_vulns if v.cvss_score >= CVSS_HIGH_THRESHOLD]
    if high_sev:
        print(
            f"   ALERT {len(high_sev)} Schwachstelle(n)"
            f" mit CVSS >= {CVSS_HIGH_THRESHOLD}!"
        )

    result = ScanResult(
        timestamp=datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        repository=REPO_NAME,
        dependencies_scanned=len(all_deps),
        vulnerabilities_found=len(all_vulns),
        high_severity_count=len(high_sev),
        dependencies=all_deps,
        vulnerabilities=all_vulns,
    )

    return result


def main() -> None:
    parser = argparse.ArgumentParser(
        description="CVE-Scanner fuer schunk_force_torque_sensor Abhaengigkeiten"
    )
    parser.add_argument(
        "--json-only",
        action="store_true",
        help="Nur JSON auf stdout ausgeben, keine Dateien schreiben",
    )
    args = parser.parse_args()

    result = run_scan()

    if args.json_only:
        print(json.dumps(asdict(result), indent=2, ensure_ascii=False))
        return

    json_path, md_path = save_reports(result, REPORT_DIR)
    print()
    print(f"JSON-Report: {json_path.relative_to(ROOT_DIR)}")
    print(f"Markdown-Report: {md_path.relative_to(ROOT_DIR)}")

    write_alert_output(result, REPORT_DIR)
    print()

    if result.vulnerabilities_found > 0:
        print(
            f"{result.vulnerabilities_found} Schwachstelle(n) gefunden! "
            "Details im Report."
        )
        if result.high_severity_count > 0:
            print(
                f"{result.high_severity_count} davon mit CVSS >= {CVSS_HIGH_THRESHOLD} "
                "– Alert wird ausgeloest."
            )
        sys.exit(1)
    else:
        print("Keine bekannten Schwachstellen – alles sicher.")
        sys.exit(0)


if __name__ == "__main__":
    main()
