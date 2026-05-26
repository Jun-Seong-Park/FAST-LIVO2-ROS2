#!/usr/bin/env bash
# check_gps_sync.sh — MID-360 GPS 시간동기 서비스 상태 검증 (read-only)                                                           
                                                                        
SVC='livox-gps-sync.service'
TAG='[sensor_bringup]'                                                   
                                                                        
state="$(systemctl is-active "$SVC")"
echo "$TAG $SVC: $state"
                                                                        
if [ "$state" != "active" ]; then
    echo "$TAG WARN: GPS time sync inactive — run 'lidar-on' or:"        
    echo "$TAG        sudo systemctl start $SVC"                       
fi 