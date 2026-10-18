#!/usr/bin/env python3
"""Check a local SIM robot started with ./gradlew simulateJava -PntCheck.

Requires scripts/sim-check-requirements.txt. Controls only localhost HAL simulation;
observes outputs through an independent NT4 client. Always disables on exit.
"""

import json
import math
from pathlib import Path
import threading
import time

import ntcore
import websocket


def main():
    nt = ntcore.NetworkTableInstance.create()
    nt.setServer("127.0.0.1")
    nt.startClient4("16th-note-live-check")
    subscription = ntcore.MultiSubscriber(
        nt, ["/AdvantageKit/", "/SmartDashboard/"],
        ntcore.PubSubOptions(periodic=0.02),
    )
    ws = websocket.create_connection("ws://127.0.0.1:3300/wpilibws", timeout=2)
    results = []

    def drain():
        try:
            while ws.recv():
                pass
        except (websocket.WebSocketException, OSError):
            pass

    threading.Thread(target=drain, daemon=True).start()

    def send(kind, device, data):
        ws.send(json.dumps({"type": kind, "device": device, "data": data}))

    def snapshot():
        base = "/AdvantageKit/TankDrive/"
        output = nt.getEntry("/AdvantageKit/RealOutputs/TankDrive/CommandedVolts")
        return {
            "requested": output.getDoubleArray([]),
            "output_timestamp": output.getLastChange(),
            **{
                key: nt.getEntry(base + key).getDouble(float("nan"))
                for key in [
                    "LeftAppliedVolts", "RightAppliedVolts", "LeftPositionMeters",
                    "RightPositionMeters", "LeftVelocityMetersPerSecond",
                    "RightVelocityMetersPerSecond", "HeadingRadians",
                ]
            },
            "feedback": nt.getEntry(base + "HasPositionFeedback").getBoolean(False),
            "enabled": nt.getEntry("/AdvantageKit/DriverStation/Enabled").getBoolean(False),
        }

    def phase(name, expected, *, throttle=0, steering=0, right_trigger=False,
              left_bumper=False, right_bumper=False,
              enabled=True, auto=False, test=False):
        deadline = time.monotonic() + 0.9
        while time.monotonic() < deadline:
            # The unnamed HAL joystick uses the configured Xbox fallback. Xbox
            # right-X is axis 4 and right trigger is axis 3.
            buttons = [False] * 14
            buttons[4] = left_bumper
            buttons[5] = right_bumper
            send("Joystick", "0", {">axes": [0, -throttle, 0, int(right_trigger), -steering, 0],
                                   ">buttons": buttons,
                                   ">povs": [-1]})
            send("DriverStation", "", {
                ">enabled": enabled, ">autonomous": auto, ">test": test,
                ">ds": True, ">new_data": True,
            })
            time.sleep(0.02)
        actual = snapshot()
        checks = {
            "connected": nt.isConnected(),
            "feedback": actual["feedback"],
            "enabled": actual["enabled"] == enabled,
            "requested": len(actual["requested"]) == 2 and all(
                abs(a - b) < 0.02 for a, b in zip(actual["requested"], expected)
            ),
            "applied": all(
                abs(actual[key] - value) < 0.02
                for key, value in zip(["LeftAppliedVolts", "RightAppliedVolts"], expected)
            ),
            "finite_feedback": all(math.isfinite(actual[key]) for key in [
                "LeftPositionMeters", "RightPositionMeters", "HeadingRadians",
                "LeftVelocityMetersPerSecond", "RightVelocityMetersPerSecond",
            ]),
        }
        # Allow deceleration after changing direction, but require the eventual
        # velocity sign and heading change to match the requested physical motion.
        for side, value in zip(["Left", "Right"], expected):
            if value != 0:
                checks[side + "_motion"] = actual[side + "VelocityMetersPerSecond"] * value > 0.1
        actual.update(name=name, expected=expected, checks=checks, passed=all(checks.values()))
        results.append(actual)
        print(f"{'PASS' if actual['passed'] else 'FAIL'} {name}: {actual['requested']}", flush=True)

    try:
        deadline = time.monotonic() + 5
        while not snapshot()["feedback"] and time.monotonic() < deadline:
            time.sleep(0.05)
        if not nt.isConnected() or not snapshot()["feedback"]:
            raise RuntimeError("Local NT4 robot with simulated tank feedback was not found")
        phase("disabled", [0, 0], enabled=False, throttle=1)
        phase("arcade forward", [12, 12], throttle=1)
        phase("arcade reverse", [-12, -12], throttle=-1)
        phase("arcade turn", [-12, 12], steering=1)
        phase("half forward uses squared input", [3, 3], throttle=0.51)
        phase("forward with turn mixes sides", [0, 12], throttle=1, steering=1)
        phase("deadband", [0, 0], throttle=0.01, steering=-0.01)
        phase("right trigger has no drive effect", [0, 0], right_trigger=True)
        phase("left bumper has no drive effect", [0, 0], left_bumper=True)
        phase("right bumper has no drive effect", [0, 0], right_bumper=True)
        phase("autonomous holds stopped", [0, 0], auto=True, throttle=1, right_trigger=True)
        phase("test holds stopped", [0, 0], test=True, throttle=1)
        phase("teleop resumes", [12, 12], throttle=1)
        phase("disable stops", [0, 0], enabled=False, throttle=1)
    finally:
        try:
            send("DriverStation", "", {">enabled": False, ">new_data": True})
            time.sleep(0.1)
        finally:
            ws.close()
            subscription.close()
            nt.stopClient()
            report = Path(__file__).resolve().parents[1] / "build/sim-networktables-results.json"
            report.parent.mkdir(parents=True, exist_ok=True)
            report.write_text(json.dumps(results, indent=2) + "\n")
            print(f"Report: {report}")
    if len(results) != 14 or not all(result["passed"] for result in results):
        raise SystemExit("Live simulation checks failed; see the JSON report")
    print("All 14 live NetworkTables checks passed.")


if __name__ == "__main__":
    main()
