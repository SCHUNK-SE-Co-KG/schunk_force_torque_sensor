# CVE-Scan Report – schunk_force_torque_sensor

**Scan-Zeitpunkt:** 2026-02-18T14:01:00Z
**Repository:** SCHUNK-SE-Co-KG/schunk_force_torque_sensor
**Abhängigkeiten geprüft:** 71
**Schwachstellen gefunden:** 5

> 5 Schwachstelle(n) gefunden!

## Zusammenfassung nach Ökosystem

| Ökosystem | Abhängigkeiten | Schwachstellen |
|-----------|---------------|----------------|
| PyPI (Python) | 22 | 3 |
| crates.io (Rust) | 40 | 2 |
| ROS 2 | 9 | 0 |
| **Gesamt** | **71** | **5** |

## Geprüfte Abhängigkeiten

### Python (PyPI)

| Paket | Version | Quelle |
|-------|---------|--------|
| black | 25.1.0 | schunk_fts_driver\setup.py |
| certifi | 2025.4.26 | schunk_fts_driver\setup.py |
| charset-normalizer | 3.4.2 | schunk_fts_driver\setup.py |
| click | 8.2.1 | schunk_fts_driver\setup.py |
| exceptiongroup | 1.3.0 | schunk_fts_driver\setup.py |
| idna | 3.10 | schunk_fts_driver\setup.py |
| iniconfig | 2.1.0 | schunk_fts_driver\setup.py |
| lark | 1.2.2 | schunk_fts_driver\setup.py |
| mypy_extensions | 1.1.0 | schunk_fts_driver\setup.py |
| numpy | 2.2.6 | schunk_fts_driver\setup.py |
| packaging | 25.0 | schunk_fts_driver\setup.py |
| pathspec | 0.12.1 | schunk_fts_driver\setup.py |
| platformdirs | 4.3.8 | schunk_fts_driver\setup.py |
| pluggy | 1.5.0 | schunk_fts_driver\setup.py |
| pytest | 8.3.5 | schunk_fts_driver\setup.py |
| pytest-repeat | 0.9.4 | schunk_fts_driver\setup.py |
| PyYAML | 6.0.2 | schunk_fts_driver\setup.py |
| requests | 2.32.4 | schunk_fts_driver\setup.py |
| tomli | 2.2.1 | schunk_fts_driver\setup.py |
| typing_extensions | 4.13.2 | schunk_fts_driver\setup.py |
| urllib3 | 2.5.0 | schunk_fts_driver\setup.py |
| psutil | 5.9.0 | schunk_fts_library\setup.py |

### Rust (crates.io)

| Crate | Version | Quelle |
|-------|---------|--------|
| addr2line | 0.24.2 | schunk_fts_dummy\Cargo.lock |
| adler2 | 2.0.0 | schunk_fts_dummy\Cargo.lock |
| autocfg | 1.4.0 | schunk_fts_dummy\Cargo.lock |
| backtrace | 0.3.75 | schunk_fts_dummy\Cargo.lock |
| bitflags | 2.9.1 | schunk_fts_dummy\Cargo.lock |
| bytes | 1.10.1 | schunk_fts_dummy\Cargo.lock |
| cfg-if | 1.0.0 | schunk_fts_dummy\Cargo.lock |
| gimli | 0.31.1 | schunk_fts_dummy\Cargo.lock |
| libc | 0.2.172 | schunk_fts_dummy\Cargo.lock |
| lock_api | 0.4.12 | schunk_fts_dummy\Cargo.lock |
| memchr | 2.7.4 | schunk_fts_dummy\Cargo.lock |
| miniz_oxide | 0.8.8 | schunk_fts_dummy\Cargo.lock |
| mio | 1.0.3 | schunk_fts_dummy\Cargo.lock |
| object | 0.36.7 | schunk_fts_dummy\Cargo.lock |
| parking_lot | 0.12.3 | schunk_fts_dummy\Cargo.lock |
| parking_lot_core | 0.9.10 | schunk_fts_dummy\Cargo.lock |
| pin-project-lite | 0.2.16 | schunk_fts_dummy\Cargo.lock |
| proc-macro2 | 1.0.95 | schunk_fts_dummy\Cargo.lock |
| quote | 1.0.40 | schunk_fts_dummy\Cargo.lock |
| redox_syscall | 0.5.12 | schunk_fts_dummy\Cargo.lock |
| rustc-demangle | 0.1.24 | schunk_fts_dummy\Cargo.lock |
| scopeguard | 1.2.0 | schunk_fts_dummy\Cargo.lock |
| signal-hook-registry | 1.4.5 | schunk_fts_dummy\Cargo.lock |
| smallvec | 1.15.0 | schunk_fts_dummy\Cargo.lock |
| socket2 | 0.5.9 | schunk_fts_dummy\Cargo.lock |
| syn | 2.0.101 | schunk_fts_dummy\Cargo.lock |
| tokio | 1.45.0 | schunk_fts_dummy\Cargo.lock |
| tokio-macros | 2.5.0 | schunk_fts_dummy\Cargo.lock |
| unicode-ident | 1.0.18 | schunk_fts_dummy\Cargo.lock |
| wasi | 0.11.0+wasi-snapshot-preview1 | schunk_fts_dummy\Cargo.lock |
| windows-sys | 0.52.0 | schunk_fts_dummy\Cargo.lock |
| windows-targets | 0.52.6 | schunk_fts_dummy\Cargo.lock |
| windows_aarch64_gnullvm | 0.52.6 | schunk_fts_dummy\Cargo.lock |
| windows_aarch64_msvc | 0.52.6 | schunk_fts_dummy\Cargo.lock |
| windows_i686_gnu | 0.52.6 | schunk_fts_dummy\Cargo.lock |
| windows_i686_gnullvm | 0.52.6 | schunk_fts_dummy\Cargo.lock |
| windows_i686_msvc | 0.52.6 | schunk_fts_dummy\Cargo.lock |
| windows_x86_64_gnu | 0.52.6 | schunk_fts_dummy\Cargo.lock |
| windows_x86_64_gnullvm | 0.52.6 | schunk_fts_dummy\Cargo.lock |
| windows_x86_64_msvc | 0.52.6 | schunk_fts_dummy\Cargo.lock |

### ROS 2

| Paket | Ökosystem | Quelle | Upstream |
|-------|-----------|--------|----------|
| rclpy | ROS | schunk_fts_driver\package.xml | [ros2/rclpy](https://github.com/ros2/rclpy) |
| launch | ROS | schunk_fts_driver\package.xml | [ros2/launch](https://github.com/ros2/launch) |
| launch_ros | ROS | schunk_fts_driver\package.xml | [ros2/launch_ros](https://github.com/ros2/launch_ros) |
| geometry_msgs | ROS | schunk_fts_driver\package.xml | [ros2/common_interfaces](https://github.com/ros2/common_interfaces) |
| std_srvs | ROS | schunk_fts_driver\package.xml | [ros2/common_interfaces](https://github.com/ros2/common_interfaces) |
| sensor_msgs | ROS | schunk_fts_driver\package.xml | [ros2/common_interfaces](https://github.com/ros2/common_interfaces) |
| diagnostic_msgs | ROS | schunk_fts_driver\package.xml | [ros2/common_interfaces](https://github.com/ros2/common_interfaces) |
| example_interfaces | ROS | schunk_fts_driver\package.xml | [ros2/example_interfaces](https://github.com/ros2/example_interfaces) |
| std_msgs | ROS | schunk_fts_interfaces\package.xml | [ros2/common_interfaces](https://github.com/ros2/common_interfaces) |

## Gefundene Schwachstellen

### GHSA-2xpw-w6gg-jr37

- **Paket:** PyPI:urllib3@2.5.0
- **CVSS-Score:** 8.9 (KRITISCH)
- **Schweregrad:** CVSS:4.0/AV:N/AC:L/AT:P/PR:N/UI:N/VC:N/VI:N/VA:H/SC:N/SI:N/SA:H
- **CVE:** CVE-2025-66471
- **Beschreibung:** urllib3 streaming API improperly handles highly compressed data
- **Fix-Version:** 2.6.0
- **Referenzen:**
  - https://github.com/urllib3/urllib3/security/advisories/GHSA-2xpw-w6gg-jr37
  - https://nvd.nist.gov/vuln/detail/CVE-2025-66471
  - https://github.com/urllib3/urllib3/commit/c19571de34c47de3a766541b041637ba5f716ed7
  - https://github.com/urllib3/urllib3

### GHSA-38jv-5279-wg99

- **Paket:** PyPI:urllib3@2.5.0
- **CVSS-Score:** 8.9 (KRITISCH)
- **Schweregrad:** CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:N/I:N/A:H
- **CVE:** CVE-2026-21441
- **Beschreibung:** Decompression-bomb safeguards bypassed when following HTTP redirects (streaming API)
- **Fix-Version:** 2.6.3
- **Referenzen:**
  - https://github.com/urllib3/urllib3/security/advisories/GHSA-38jv-5279-wg99
  - https://nvd.nist.gov/vuln/detail/CVE-2026-21441
  - https://github.com/urllib3/urllib3/commit/8864ac407bba8607950025e0979c4c69bc7abc7b
  - https://github.com/urllib3/urllib3
  - https://lists.debian.org/debian-lts-announce/2026/01/msg00017.html

### GHSA-gm62-xv2j-4w53

- **Paket:** PyPI:urllib3@2.5.0
- **CVSS-Score:** 8.9 (KRITISCH)
- **Schweregrad:** CVSS:4.0/AV:N/AC:L/AT:P/PR:N/UI:N/VC:N/VI:N/VA:H/SC:N/SI:N/SA:H
- **CVE:** CVE-2025-66418
- **Beschreibung:** urllib3 allows an unbounded number of links in the decompression chain
- **Fix-Version:** 2.6.0
- **Referenzen:**
  - https://github.com/urllib3/urllib3/security/advisories/GHSA-gm62-xv2j-4w53
  - https://nvd.nist.gov/vuln/detail/CVE-2025-66418
  - https://github.com/urllib3/urllib3/commit/24d7b67eac89f94e11003424bcf0d8f7b72222a8
  - https://github.com/urllib3/urllib3

### GHSA-434x-w66g-qw3r

- **Paket:** crates.io:bytes@1.10.1
- **CVSS-Score:** 5.5
- **Schweregrad:** CVSS:4.0/AV:L/AC:L/AT:N/PR:N/UI:N/VC:N/VI:N/VA:H/SC:N/SI:N/SA:N/E:P
- **CVE:** CVE-2026-25541
- **Beschreibung:** bytes has integer overflow in BytesMut::reserve
- **Fix-Version:** 1.11.1
- **Referenzen:**
  - https://github.com/tokio-rs/bytes/security/advisories/GHSA-434x-w66g-qw3r
  - https://nvd.nist.gov/vuln/detail/CVE-2026-25541
  - https://github.com/tokio-rs/bytes/commit/d0293b0e35838123c51ca5dfdf468ecafee4398f
  - https://github.com/tokio-rs/bytes
  - https://github.com/tokio-rs/bytes/releases/tag/v1.11.1

### RUSTSEC-2026-0007

- **Paket:** crates.io:bytes@1.10.1
- **CVSS-Score:** 5.5
- **Schweregrad:** UNKNOWN
- **CVE:** CVE-2026-25541
- **Beschreibung:** Integer overflow in `BytesMut::reserve`
- **Fix-Version:** 1.11.1
- **Referenzen:**
  - https://crates.io/crates/bytes
  - https://rustsec.org/advisories/RUSTSEC-2026-0007.html
  - https://github.com/advisories/GHSA-434x-w66g-qw3r

---
*Automatisch generiert von `security/cve_scanner.py` via [OSV.dev](https://osv.dev) und GitHub Advisory Database.*
