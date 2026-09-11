# Runs inside the container: start PX4 SITL with SIH, then fly it over MAVLink.
apt-get -qq update >/dev/null 2>&1
apt-get -qq install -y python3 python3-pip >/dev/null 2>&1
pip3 install -q --break-system-packages pymavlink >/dev/null 2>&1
cd /px4
# 10040 = sih_quadx. PX4 integrates its own rigid-body model at 250 Hz, so no
# external simulator is needed — which is what makes this work on arm64.
PX4_SYS_AUTOSTART=10040 ./build/px4_sitl_default/bin/px4 -d \
    -s etc/init.d-posix/rcS ROMFS/px4fmu_common > /tmp/px4.log 2>&1 &
sleep 35
python3 /scripts/fly_px4.py
