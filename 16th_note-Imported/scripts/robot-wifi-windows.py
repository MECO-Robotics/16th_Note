#!/usr/bin/env python3
"""Recover and verify this workstation's robot Wi-Fi adapter inside Windows."""

import base64
import json
import re
import subprocess
import sys
import time

VM = "frc2026-8324"
ADAPTER_MAC = "d4:0d:ab:04:69:d4"
DEVICE_ID = r"USB\VID_0BDA&PID_C811\123456"


def agent(command, arguments=None):
    request = {"execute": command}
    if arguments is not None:
        request["arguments"] = arguments
    result = subprocess.run(
        ["virsh", "-c", "qemu:///system", "qemu-agent-command", VM, json.dumps(request)],
        check=True, capture_output=True, text=True, timeout=10,
    )
    response = json.loads(result.stdout)
    if "error" in response:
        raise RuntimeError(str(response["error"]))
    return response["return"]


def guest_run(path, args, capture=False):
    pid = agent("guest-exec", {"path": path, "arg": args, "capture-output": True})["pid"]
    for _ in range(25):
        result = agent("guest-exec-status", {"pid": pid})
        if result.get("exited"):
            output = "".join(
                base64.b64decode(result.get(key, "")).decode(errors="replace")
                for key in ("out-data", "err-data")
            )
            if capture:
                return output
            print(output.strip(), flush=True)
            return result.get("exitcode", 1)
        time.sleep(1)
    raise RuntimeError("Windows command timed out; check the VM task terminal.")


def robot_address():
    # Guest-agent addresses can be stale after WLAN disconnects.
    status = guest_run(r"C:\Windows\System32\netsh.exe", ["wlan", "show", "interfaces"], capture=True)
    if "location permission" in status.lower():
        raise RuntimeError("Windows blocks WLAN queries: connect to 8324 from the Windows Wi-Fi menu, or enable Settings > Privacy & security > Location and retry.")
    if not re.search(r"State\s*:\s*connected\s*$", status, re.MULTILINE):
        return None
    if not re.search(r"^\s*SSID\s*:\s*8324\s*$", status, re.MULTILINE):
        return None
    for interface in agent("guest-network-get-interfaces"):
        if interface.get("hardware-address", "").lower() == ADAPTER_MAC:
            for address in interface.get("ip-addresses", []):
                value = address.get("ip-address", "")
                if address.get("ip-address-type") == "ipv4" and value.startswith("10.83.24."):
                    return value
    return None


def main():
    agent("guest-ping")
    address = robot_address()
    if not address:
        print("Waiting for Windows robot Wi-Fi...", flush=True)
        for _ in range(5):
            time.sleep(1)
            address = robot_address()
            if address:
                break
    if not address:
        print("Restarting the Realtek adapter to recover driver startup failures...", flush=True)
        if guest_run(r"C:\Windows\System32\pnputil.exe", ["/restart-device", DEVICE_ID]):
            raise RuntimeError("Windows could not restart the Realtek adapter.")
        # Use the saved profile; no credentials are read or changed.
        if guest_run(r"C:\Windows\System32\netsh.exe", ["wlan", "connect", "name=8324"]):
            raise RuntimeError("Windows rejected the saved Wi-Fi connection request. If the output mentions location permission, connect to 8324 in the Windows Wi-Fi menu or enable Location services and retry.")
        for _ in range(20):
            address = robot_address()
            if address:
                break
            time.sleep(1)
    if not address:
        raise RuntimeError("USB attached, but Windows has no robot Wi-Fi address. Check Device Manager and connect to 8324 in Windows.")
    print(f"Windows robot Wi-Fi address: {address}", flush=True)
    if guest_run(r"C:\Windows\System32\PING.EXE", ["-n", "2", "-w", "1500", "10.83.24.2"]):
        raise RuntimeError("Windows Wi-Fi connected, but roboRIO did not answer. Check robot power and Driver Station.")
    print("Windows can reach roboRIO. Check Driver Station communication before enabling.")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, subprocess.SubprocessError, KeyError, ValueError) as error:
        print(f"Wi-Fi verification failed: {error}", file=sys.stderr)
        sys.exit(1)
