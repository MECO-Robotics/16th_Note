#!/usr/bin/env python3
"""Drive the live desktop robot through a three-source fuel and scoring route.

Start the robot with ``./gradlew simulateJava -PntCheck``. This client uses the
same HAL joystick and Driver Station inputs as the normal driver controls and
reads only published NetworkTables telemetry. It keeps the robot center and its
conservative chassis radius outside the two blue bump footprints.
"""

import json
import math
from pathlib import Path
import threading
import time

import ntcore
import websocket


HUB_X = 4.625467
HUB_Y = 4.034663
ROBOT_RADIUS = 0.32
BUMP_HALF_WIDTH = 1.854 / 2
BUMP_DEPTH = 1.128
HUB_HALF_WIDTH = 0.60
SHOT = (3.304737959, HUB_Y)
DEPOT = (0.75, 6.95)
OUTPOST = (0.75, 0.75)
NEUTRAL = ((HUB_X + 11.915521) / 2, HUB_Y)


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    nt = ntcore.NetworkTableInstance.create()
    nt.setServer("127.0.0.1")
    nt.startClient4("16th-note-match-route")
    subscription = ntcore.MultiSubscriber(
        nt, ["/AdvantageKit/", "/SmartDashboard/"],
        ntcore.PubSubOptions(periodic=0.02),
    )
    ws = websocket.create_connection("ws://127.0.0.1:3300/wpilibws", timeout=2)
    path = []
    source_results = []
    monitor_bump = False

    def drain():
        try:
            while ws.recv():
                pass
        except (websocket.WebSocketException, OSError):
            pass

    threading.Thread(target=drain, daemon=True).start()

    def send(kind, device, data):
        ws.send(json.dumps({"type": kind, "device": device, "data": data}))

    def number(key, default=float("nan")):
        return nt.getEntry("/SmartDashboard/Fuel/Sim/" + key).getDouble(default)

    def pose():
        return number("PoseX"), number("PoseY"), math.radians(number("HeadingDegrees"))

    def in_expanded_bump(x, y):
        x_low = HUB_X - BUMP_HALF_WIDTH - ROBOT_RADIUS
        x_high = HUB_X + BUMP_HALF_WIDTH + ROBOT_RADIUS
        lower = (
            HUB_Y - HUB_HALF_WIDTH - BUMP_DEPTH - ROBOT_RADIUS,
            HUB_Y - HUB_HALF_WIDTH + ROBOT_RADIUS,
        )
        upper = (
            HUB_Y + HUB_HALF_WIDTH - ROBOT_RADIUS,
            HUB_Y + HUB_HALF_WIDTH + BUMP_DEPTH + ROBOT_RADIUS,
        )
        return x_low <= x <= x_high and (
            lower[0] <= y <= lower[1] or upper[0] <= y <= upper[1]
        )

    def tick(*, forward=0, turn=0, intake=False, shoot=False, enabled=True):
        buttons = [False] * 14
        buttons[4] = intake  # Xbox LB
        buttons[5] = shoot   # Xbox RB
        send(
            "Joystick",
            "0",
            {
                ">axes": [0, -forward, 0, 0, -turn, 0],
                ">buttons": buttons,
                ">povs": [-1],
            },
        )
        send(
            "DriverStation",
            "",
            {
                ">enabled": enabled,
                ">autonomous": False,
                ">test": False,
                ">ds": True,
                ">new_data": True,
            },
        )
        time.sleep(0.02)
        current = pose()
        if all(math.isfinite(v) for v in current):
            if monitor_bump and in_expanded_bump(current[0], current[1]):
                raise RuntimeError(
                    f"route entered expanded bump footprint at ({current[0]:.3f}, {current[1]:.3f})"
                )
            path.append([current[0], current[1], math.degrees(current[2])])

    def wait_ticks(count, **controls):
        for _ in range(count):
            tick(**controls)

    def rotate_to(heading, *, intake=False, timeout=20):
        deadline = time.monotonic() + timeout
        stable = 0
        previous_heading = pose()[2]
        while time.monotonic() < deadline:
            current_heading = pose()[2]
            error = wrap(heading - current_heading)
            angular_rate = wrap(current_heading - previous_heading) / 0.02
            previous_heading = current_heading
            if abs(error) < 0.035 and abs(angular_rate) < 0.06:
                stable += 1
                tick(intake=intake)
                if stable >= 12:
                    return
            else:
                stable = 0
                # Compensate for WPILib's squared manual inputs after applying
                # proportional heading and angular-rate braking.
                linear_turn = clamp(0.12 * error - 0.08 * angular_rate, -0.08, 0.08)
                turn = math.copysign(math.sqrt(abs(linear_turn)), linear_turn)
                tick(turn=turn, intake=intake)
        raise RuntimeError(f"timed out rotating to {math.degrees(heading):.1f} degrees")

    def drive_to(target, *, intake=False, tolerance=0.08, timeout=25):
        x, y, _ = pose()
        if math.hypot(target[0] - x, target[1] - y) < tolerance:
            wait_ticks(8, intake=intake)
            return
        rotate_to(math.atan2(target[1] - y, target[0] - x), intake=intake)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            x, y, heading = pose()
            dx, dy = target[0] - x, target[1] - y
            distance = math.hypot(dx, dy)
            if distance < tolerance:
                wait_ticks(8, intake=intake)
                return
            error = wrap(math.atan2(dy, dx) - heading)
            if abs(error) > 0.35:
                rotate_to(math.atan2(dy, dx), intake=intake)
                continue
            turn = clamp(error * 0.08, -0.08, 0.08)
            forward = clamp(distance * 0.28, 0.14, 0.42)
            tick(forward=forward, turn=turn, intake=intake)
        raise RuntimeError(f"timed out driving to {target}; pose is {pose()}")

    def waypoints(*points):
        for point in points:
            drive_to(point)

    def collect(name, source, approach, heading):
        before = int(number("HeldCount", -1))
        drive_to(approach)
        rotate_to(heading, intake=True)
        deadline = time.monotonic() + 8
        while number("HeldCount", -1) < 6 and time.monotonic() < deadline:
            tick(forward=0.15, intake=True)
        wait_ticks(20)
        held = int(number("HeldCount", -1))
        if before != 0 or held != 6:
            raise RuntimeError(f"{name} pickup expected 0->6 held fuel, observed {before}->{held}")
        x, y, _ = pose()
        if math.hypot(x - source[0], y - source[1]) > 0.8:
            raise RuntimeError(f"{name} pickup occurred outside its source area")
        source_results.append({"source": name, "held": held, "pickupPose": [x, y]})
        print(f"PASS collected six from {name} at ({x:.2f}, {y:.2f})", flush=True)

    def score(expected_total):
        drive_to(SHOT, tolerance=0.035)
        rotate_to(math.pi)
        deadline = time.monotonic() + 10
        while number("ScoredCount", -1) < expected_total and time.monotonic() < deadline:
            tick(shoot=True)
        wait_ticks(25)
        scored = int(number("ScoredCount", -1))
        launched = int(number("LaunchedCount", -1))
        if scored != expected_total or launched != expected_total or number("HeldCount", -1) != 0:
            raise RuntimeError(
                f"score cycle expected {expected_total}; scored={scored}, launched={launched}, held={number('HeldCount', -1)}"
            )
        print(f"PASS scored {expected_total}/18 cumulative fuel", flush=True)

    report = Path(__file__).resolve().parents[1] / "build/sim-match-route-results.json"
    outcome = {"passed": False}
    try:
        deadline = time.monotonic() + 5
        while (not nt.isConnected() or not math.isfinite(number("PoseX"))) and time.monotonic() < deadline:
            time.sleep(0.05)
        if not nt.isConnected():
            raise RuntimeError("local NT4 robot was not found")

        wait_ticks(10, enabled=False)
        command = nt.getEntry(
            "/SmartDashboard/Fuel/Sim/Prepare full-field route (disabled)/running"
        )
        command.setBoolean(True)
        nt.flush()
        wait_ticks(25, enabled=False)
        command.setBoolean(False)
        nt.flush()
        if int(number("GroundCount", -1)) != 18:
            status = nt.getEntry("/SmartDashboard/Fuel/Sim/ResetStatus").getString("")
            raise RuntimeError(f"route preparation failed: ground={number('GroundCount', -1)}, status={status}")
        path.clear()
        monitor_bump = True

        # Enable with every control released to satisfy the mechanism re-arm interlock.
        wait_ticks(20)
        collect("depot", DEPOT, (1.50, DEPOT[1]), math.pi)
        waypoints((2.5, 6.95), (3.0, 5.8), (3.0, HUB_Y), SHOT)
        score(6)

        waypoints((3.0, 2.0), (1.50, OUTPOST[1]))
        collect("outpost", OUTPOST, (1.50, OUTPOST[1]), math.pi)
        waypoints((3.0, 2.0), (3.0, HUB_Y), SHOT)
        score(12)

        waypoints((3.0, 6.5), (4.6, 7.0), (NEUTRAL[0], 7.0), (NEUTRAL[0], 5.0))
        collect("neutral zone", NEUTRAL, (NEUTRAL[0], 5.0), -math.pi / 2)
        waypoints((NEUTRAL[0], 7.0), (4.6, 7.0), (3.0, 6.5), (3.0, HUB_Y), SHOT)
        score(18)

        outcome = {
            "passed": True,
            "sources": source_results,
            "scored": int(number("ScoredCount", -1)),
            "launched": int(number("LaunchedCount", -1)),
            "groundRemaining": int(number("GroundCount", -1)),
            "pathSamples": len(path),
            "avoidedExpandedBumpFootprints": True,
        }
        print("All depot, outpost, neutral-zone, bump-avoidance, and scoring checks passed.")
    except Exception as error:
        outcome = {"passed": False, "error": str(error), "sources": source_results}
        raise
    finally:
        try:
            send("DriverStation", "", {">enabled": False, ">ds": True, ">new_data": True})
            time.sleep(0.1)
        finally:
            outcome["path"] = path
            report.parent.mkdir(parents=True, exist_ok=True)
            report.write_text(json.dumps(outcome, indent=2) + "\n")
            ws.close()
            subscription.close()
            nt.stopClient()
            print(f"Report: {report}")


if __name__ == "__main__":
    main()
