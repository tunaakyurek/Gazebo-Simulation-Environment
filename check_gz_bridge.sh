#!/bin/bash

echo "=== CHECKING GZ_BRIDGE STATUS ==="
echo "This script will check the gz_bridge functionality"
echo "Please run this with: sudo -u wa.akyuret1 bash check_gz_bridge.sh"
echo ""

echo "1. Checking gz_bridge executable:"
ls -la /u/88/wa.akyuret1/unix/PX4-Autopilot/build/px4_sitl_default/bin/px4-gz_bridge

echo -e "\n2. Testing gz_bridge help:"
cd /u/88/wa.akyuret1/unix/PX4-Autopilot
./build/px4_sitl_default/bin/px4-gz_bridge --help

echo -e "\n3. Checking if gz_bridge is in PATH:"
which gz_bridge || echo "gz_bridge not in PATH"

echo -e "\n4. Creating gz_bridge symlink:"
ln -sf /u/88/wa.akyuret1/unix/PX4-Autopilot/build/px4_sitl_default/bin/px4-gz_bridge /usr/local/bin/gz_bridge
echo "gz_bridge symlink created"

echo -e "\n5. Verifying gz_bridge is now available:"
which gz_bridge
