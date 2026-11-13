#!/bin/sh
# BMS772 Monitor - optional one-time setup.
#
# Makes Linux start the USB-to-CAN adapter at the BMS bitrate whenever it is
# plugged in, so the monitor never has to ask for a password, and adds a
# "BMS772 Monitor" entry to the applications menu.
#
#   ./install.sh               install (asks for the sudo password once)
#   ./install.sh 500000        install for a different bitrate
#   ./install.sh --print       show the udev rule without installing anything
#   ./install.sh --uninstall   remove it again
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
RULE=/etc/udev/rules.d/80-bms772-monitor.rules
BITRATE=1000000
ACTION=install

for arg in "$@"; do
    case "$arg" in
        --print) ACTION=print ;;
        --uninstall) ACTION=uninstall ;;
        *[!0-9]*|'') echo "Unknown option: $arg" >&2; exit 2 ;;
        *) BITRATE=$arg ;;
    esac
done

IP=$(command -v ip || true)
for candidate in /usr/sbin/ip /sbin/ip /usr/bin/ip; do
    [ -n "$IP" ] || { [ -x "$candidate" ] && IP=$candidate; } || true
done

# gs_usb is the kernel driver for the Innomaker USB2CAN and other candleLight
# compatible adapters. The rule only touches adapters that use it.
rule() {
    printf '%s\n' "# Installed by BMS772 Monitor (tools/bms-monitor/install.sh)"
    printf '%s\n' "ACTION==\"add\", SUBSYSTEM==\"net\", KERNEL==\"can*\", DRIVERS==\"gs_usb\", RUN+=\"/bin/sh -c '$IP link set %k type can bitrate $BITRATE restart-ms 100 && $IP link set %k up'\""
}

if [ "$ACTION" = print ]; then
    [ -n "$IP" ] || IP=/usr/sbin/ip
    rule
    exit 0
fi

USER_NAME=${SUDO_USER:-$(id -un)}
USER_HOME=$(getent passwd "$USER_NAME" 2>/dev/null | cut -d: -f6)
[ -n "$USER_HOME" ] || USER_HOME=$HOME
LAUNCHER="$USER_HOME/.local/share/applications/bms772-monitor.desktop"

if [ "$(id -u)" -ne 0 ]; then
    echo "Administrator rights are needed to change the udev rules."
    exec sudo sh "$0" "$@"
fi

if [ "$ACTION" = uninstall ]; then
    rm -f "$RULE" "$LAUNCHER"
    udevadm control --reload-rules 2>/dev/null || true
    echo "Removed."
    exit 0
fi

if [ -z "$IP" ]; then
    echo "The 'ip' command was not found. Install the iproute2 package and run this again." >&2
    exit 1
fi

rule > "$RULE"
udevadm control --reload-rules
udevadm trigger --subsystem-match=net --action=add
echo "Installed $RULE (adapter starts at $BITRATE bit/s when plugged in)."

mkdir -p "$(dirname "$LAUNCHER")"
cat > "$LAUNCHER" <<DESKTOP
[Desktop Entry]
Type=Application
Name=BMS772 Monitor
Comment=Live battery data from the RDDRONE-BMS772 over USB-to-CAN
Exec=python3 "$HERE/bms_monitor.py"
Terminal=true
Categories=Utility;Electronics;
DESKTOP
chown "$USER_NAME" "$LAUNCHER" 2>/dev/null || true
echo "Added 'BMS772 Monitor' to the applications menu."
echo "Unplug the adapter and plug it back in, then run: python3 $HERE/bms_monitor.py"
