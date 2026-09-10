#!/usr/bin/env bash
set -euo pipefail
vm=frc2026-8324
iface=wlxd40dab0469d4
profile=frc8324-ubuntu-wifi
xml=/home/brian/Documents/roborio-8324-2026/network/wifi-usb.xml
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
owner() {
 virsh -c qemu:///system dumpxml "$vm" | python3 -c 'import sys,xml.etree.ElementTree as E; r=E.parse(sys.stdin); print("windows" if any(n.find("source/vendor") is not None and n.find("source/vendor").get("id")=="0x0bda" and n.find("source/product").get("id")=="0xc811" for n in r.findall("./devices/hostdev")) else "ubuntu")'
}
case "${1:-status}" in
 status) echo "VM attachment owner: $(owner)"; nmcli -t -f DEVICE,STATE,CONNECTION device ;;
 ubuntu)
  if [[ $(owner) == windows ]]; then virsh -c qemu:///system detach-device "$vm" "$xml" --live; fi
  for ((i=0;i<20;i++)); do [[ -e /sys/class/net/$iface ]] && break; sleep 1; done
  [[ -e /sys/class/net/$iface ]] || { echo 'USB Wi-Fi adapter missing'; exit 1; }
  nmcli connection up "$profile" ifname "$iface"
  ip route get 10.83.24.2
  echo 'Ubuntu robot Wi-Fi ready. Windows Driver Station is disconnected.'
  ;;
 windows)
  [[ $(virsh -c qemu:///system domstate "$vm") == running ]] || { echo 'Start the Windows VM first.'; exit 1; }
  if [[ $(owner) != windows ]]; then
   [[ -e /sys/class/net/$iface ]] || { echo 'Expected USB Wi-Fi adapter missing'; exit 1; }
   nmcli connection down "$profile" || true
   virsh -c qemu:///system attach-device "$vm" "$xml" --live
  fi
  python3 "$script_dir/robot-wifi-windows.py"
  ;;
 *) echo 'Usage: robot-wifi.sh ubuntu|windows|status'; exit 2 ;;
esac
