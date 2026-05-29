#!/usr/bin/env bash
# net.core.{r,w}mem_max 를 INT_MAX(2147483647) 로 영구 설정 (재부팅 후에도 유지).
# 근거: kernel v5.15 sysctl_net_core.c — rmem_max/wmem_max 는 상한(extra2) 없는 int,
#       물리 상한 INT_MAX. fastdds_unicast.xml 의 send/receiveBufferSize=INT_MAX 가
#       런타임에 clamp 없이 적용되려면 이 cap 이 INT_MAX 여야 함.
set -u

CONF=/etc/sysctl.d/99-fastdds-sockbuf.conf

sudo tee "$CONF" >/dev/null <<'EOF'
# FastDDS unicast 대용량 소켓 버퍼용 — econ_ros2_driver/config/fastdds_unicast.xml 과 짝
net.core.rmem_max = 2147483647
net.core.wmem_max = 2147483647
EOF

sudo sysctl --system >/dev/null

echo "[적용] rmem_max=$(cat /proc/sys/net/core/rmem_max) wmem_max=$(cat /proc/sys/net/core/wmem_max)"
echo "[파일] $CONF"
