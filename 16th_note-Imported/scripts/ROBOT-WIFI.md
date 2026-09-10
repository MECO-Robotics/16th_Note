# Robot Wi-Fi ownership

In WPILib VS Code, open the Command Palette (**Ctrl+Shift+P**), select
**Tasks: Run Task** (or **Terminal → Run Task**), then:

- **Robot Wi-Fi: Ubuntu (deploy)** to connect the USB adapter to team 8324 for Gradle deployment.
- **Robot Wi-Fi: Windows (Driver Station)** to attach it to the running `frc2026-8324` VM and verify robot connectivity.
- **Robot Wi-Fi: Status** to inspect the current attachment and host connections.

Terminal equivalents: `./scripts/robot-wifi.sh ubuntu`, `windows`, or `status`.

Only one OS can own this USB adapter at a time. Switching to Ubuntu disconnects Driver Station. The built-in ToriRouter connection is untouched. Windows automatically reconnects only if its saved 8324 Wi-Fi profile permits it; verify Driver Station communication afterward.

These commands use this workstation's existing libvirt VM, USB XML at `/home/brian/Documents/roborio-8324-2026/network/wifi-usb.xml`, NetworkManager profile `frc8324-ubuntu-wifi`, interface `wlxd40dab0469d4`, and existing NordVPN exception for `10.83.24.2/32`. They do not deploy code or enable the robot. Ownership changes are live only; the VM retains its saved USB assignment for its next start.

The Windows task uses the QEMU guest agent to check the Realtek adapter’s robot-network address and ping the roboRIO. If the adapter has no robot address, it restarts the specific Realtek USB device once (recovering the observed Device Manager Code 10) and requests the saved `8324` profile. Failures leave the task terminal visible with an error; USB attachment alone is not reported as success. The guest agent must be running. A successful ping does not verify Driver Station application communication.

Windows may deny `netsh wlan` commands when Location services are off. If the task reports this, connect to `8324` through the Windows Wi-Fi menu, or enable **Settings → Privacy & security → Location** and retry. The helper does not change privacy permissions. Verification of WLAN state expects this VM’s English Windows output.
