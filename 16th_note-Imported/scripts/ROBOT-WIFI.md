# Robot Wi-Fi ownership

In WPILib VS Code, select **Terminal → Run Task**, then:

- **Robot Wi-Fi: Ubuntu (deploy)** to connect the USB adapter to team 8324 for Gradle deployment.
- **Robot Wi-Fi: Windows (Driver Station)** to attach it to the running `frc2026-8324` VM.
- **Robot Wi-Fi: Status** to inspect the current attachment and host connections.

Terminal equivalents: `./scripts/robot-wifi.sh ubuntu`, `windows`, or `status`.

Only one OS can own this USB adapter at a time. Switching to Ubuntu disconnects Driver Station. The built-in ToriRouter connection is untouched. Windows automatically reconnects only if its saved 8324 Wi-Fi profile permits it; verify Driver Station communication afterward.

These commands use this workstation's existing libvirt VM, USB XML at `/home/brian/Documents/roborio-8324-2026/network/wifi-usb.xml`, NetworkManager profile `frc8324-ubuntu-wifi`, interface `wlxd40dab0469d4`, and existing NordVPN exception for `10.83.24.2/32`. They do not deploy code or enable the robot. Ownership changes are live only; the VM retains its saved USB assignment for its next start.
