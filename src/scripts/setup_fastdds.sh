#!/usr/bin/env bash
# Permanently set net.core.{r,w}mem_max to INT_MAX (2147483647), persists across reboots.
# Rationale: kernel v5.15 sysctl_net_core.c — rmem_max/wmem_max are uncapped ints,
#            physical ceiling is INT_MAX. fastdds_unicast.xml sets send/receiveBufferSize=INT_MAX,
#            which only takes effect without clamping if this kernel cap is also INT_MAX.
set -u

CONF=/etc/sysctl.d/99-fastdds-sockbuf.conf

sudo tee "$CONF" >/dev/null <<'EOF'
# Large socket buffer for FastDDS unicast — paired with econ_ros2_driver/config/fastdds_unicast.xml
net.core.rmem_max = 2147483647
net.core.wmem_max = 2147483647
EOF

sudo sysctl --system >/dev/null

echo "[applied] rmem_max=$(cat /proc/sys/net/core/rmem_max) wmem_max=$(cat /proc/sys/net/core/wmem_max)"
echo "[file]    $CONF"
