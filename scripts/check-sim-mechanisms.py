"""Exercise mechanism bindings through the live robot's simulated Xbox controller."""

import json
from pathlib import Path
import threading
import time

import ntcore
import websocket


def main():
    nt = ntcore.NetworkTableInstance.create()
    nt.setServer("127.0.0.1")
    nt.startClient4("mechanism-gamepad-check")
    sub = ntcore.MultiSubscriber(nt, ["/AdvantageKit/"], ntcore.PubSubOptions(periodic=0.02))
    ws = websocket.create_connection("ws://127.0.0.1:3300/wpilibws", timeout=2)
    ws.settimeout(None)

    def drain():
        try:
            while ws.recv():
                pass
        except (OSError, websocket.WebSocketException):
            pass

    threading.Thread(target=drain, daemon=True).start()
    results = []

    def send(kind, device, data):
        ws.send(json.dumps({"type": kind, "device": device, "data": data}))

    def gamepad(seconds=0.4, *, buttons=(), forward=0, lt=0, rt=0, pov=-1, enabled=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            values = [index + 1 in buttons for index in range(14)]
            send("Joystick", "0", {">axes": [0, -forward, lt, rt, 0, 0],
                                   ">buttons": values, ">povs": [pov]})
            send("DriverStation", "", {">enabled": enabled, ">autonomous": False,
                                       ">test": False, ">ds": True, ">new_data": True})
            time.sleep(0.02)

    def value(mechanism, key):
        return nt.getEntry(f"/AdvantageKit/{mechanism}/{key}").getDouble(float("nan"))

    def check(name, passed, **observed):
        results.append(dict(name=name, passed=bool(passed), **observed))
        print(f"{'PASS' if passed else 'FAIL'} {name}: {observed}", flush=True)

    try:
        deadline = time.monotonic() + 10
        while not nt.isConnected() and time.monotonic() < deadline:
            time.sleep(0.05)
        gamepad(enabled=False)
        gamepad(rt=1)
        check("RT intake forward", value("Intake", "AppliedVolts") > 5.5)
        gamepad(lt=1)
        check("LT intake reverse", value("Intake", "AppliedVolts") < -5.5)
        gamepad(lt=1, rt=1)
        check("opposing intake triggers neutral", abs(value("Intake", "AppliedVolts")) < 0.01)
        gamepad(buttons=(6,))
        check("RB indexer forward", value("Indexer", "AppliedVolts") > 5.5)
        gamepad(buttons=(5,))
        check("LB indexer reverse", value("Indexer", "AppliedVolts") < -5.5)
        gamepad(buttons=(5, 6))
        check("opposing bumpers neutral", abs(value("Indexer", "AppliedVolts")) < 0.01)
        gamepad()
        check("roller release stops", abs(value("Intake", "AppliedVolts")) < 0.01
              and abs(value("Indexer", "AppliedVolts")) < 0.01)
        gamepad(buttons=(1,))
        gamepad(3)
        rpm = value("Shooter", "Velocity") * 60
        check("A holds 50 percent free speed after release",
              abs(rpm - 5676 * (22 / 26) * 0.5) < 150, rpm=rpm)
        gamepad(buttons=(2,))
        check("B zero voltage", abs(value("Shooter", "AppliedVolts")) < 0.01)
        gamepad(buttons=(4,), forward=1)
        drive_volts = nt.getEntry(
            "/AdvantageKit/RealOutputs/TankDrive/CommandedVolts"
        ).getDoubleArray([])
        shooter_volts = value("Shooter", "AppliedVolts")
        check("Y vision shooter stops without vision and preserves manual drive",
              len(drive_volts) == 2
              and all(volts > 11.5 for volts in drive_volts)
              and abs(shooter_volts) < 0.01,
              driveVolts=list(drive_volts), shooterVolts=shooter_volts)
        gamepad(buttons=(3,))
        gamepad(3)
        rpm = value("Shooter", "Velocity") * 60
        check("X holds 75 percent free speed", abs(rpm - 5676 * (22 / 26) * 0.75) < 200, rpm=rpm)
        gamepad(pov=0)
        gamepad(6)
        position = value("Pivot", "OutputPosition")
        check("POV up deploys to -60 degrees", abs(position - (-60 / 360)) < 0.03,
              rotations=position)
        gamepad(pov=90)
        gamepad(4)
        position = value("Pivot", "OutputPosition")
        check("POV right reaches +15 degrees", abs(position - (15 / 360)) < 0.03,
              rotations=position)
        gamepad(pov=180)
        gamepad(6)
        position = value("Pivot", "OutputPosition")
        check("POV down stows at +90 degrees", abs(position - 0.25) < 0.03, rotations=position)
        gamepad(buttons=(1, 6), rt=1, pov=0)
        gamepad(enabled=False)
        volts = {name: value(name, "AppliedVolts")
                 for name in ("Shooter", "Pivot", "Intake", "Indexer")}
        check("disable stops all mechanisms", all(abs(v) < 0.01 for v in volts.values()), **volts)
    finally:
        try:
            gamepad(enabled=False)
        finally:
            ws.close()
            sub.close()
            nt.stopClient()
            report = Path(__file__).resolve().parents[1] / "build/sim-mechanism-results.json"
            report.write_text(json.dumps(results, indent=2) + "\n")
            print(f"Report: {report}")
    if len(results) != 15 or not all(result["passed"] for result in results):
        raise SystemExit("Mechanism simulation checks failed")


if __name__ == "__main__":
    main()
