#!/bin/sh
# 反复强制探测 HDMI connector，观察 EDID / modes 是否能读到
for i in 1 2 3 4 5; do
  echo detect > /sys/class/drm/card0-HDMI-A-1/status 2>/dev/null
  sleep 1
  st=$(cat /sys/class/drm/card0-HDMI-A-1/status)
  n=$(cat /sys/class/drm/card0-HDMI-A-1/modes 2>/dev/null | wc -l)
  e=$(stat -c%s /sys/class/drm/card0-HDMI-A-1/edid 2>/dev/null)
  echo "try$i st=$st modes=$n edid=$e"
done
echo "--- dmesg hdmi ---"
dmesg | grep -iE "hdmi" | tail -8
