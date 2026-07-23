# Copyright 2026 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------
import importlib.util

collect_ignore = [
    "schunk_fts_driver/test",
    "schunk_fts_library/test",
]


def pytest_ignore_collect(collection_path, config):
    path = str(collection_path)
    if "schunk_fts_driver/schunk_fts_driver/tests" not in path:
        return None

    if importlib.util.find_spec("schunk_fts_interfaces.msg") is None:
        return True

    return None
