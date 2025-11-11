#/usr/bin/bash
cd $HOME
apt-get install -y socat

# Python dependencies from setup.py
os_name=$(lsb_release -cs)

case $os_name in
  jammy) # Ubuntu 22.04
    pip install --user -e /workspace/src/schunk_fts_library
    pip install --user -e /workspace/src/schunk_fts_driver
    ;;
  kinetic) # Ubuntu 24.04
    pip install --break-system-packages -e /workspace/src/schunk_fts_library
    pip install --break-system-packages -e /workspace/src/schunk_fts_driver
    ;;
  *) # Newer
    pip install --break-system-packages -e /workspace/src/schunk_fts_library
    pip install --break-system-packages -e /workspace/src/schunk_fts_driver
    ;;
esac
