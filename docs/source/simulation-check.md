# Simulation

Use the WPILib 2026 Java 17 toolchain. On this Windows workstation it is installed at `C:\Users\Public\wpilib\2026\jdk`.

Start the robot with `./gradlew.bat simulateJava -PntCheck`. The optional `ntCheck` flag enables the localhost HAL WebSocket server for the integration checker.

Install `scripts/sim-check-requirements.txt` into a Python environment, then run `scripts/check-sim-networktables.py` while the simulator is running. It exercises disabled, teleop, autonomous, and test modes, checks commanded/applied voltages and simulated motion, and disables the simulated robot on exit. Results go to `build/sim-networktables-results.json`.

The checker uses localhost NT4 on port 5810 and HAL WebSockets on port 3300. Run only one simulated Driver Station controller at a time.

The drive voltage topic is `/AdvantageKit/RealOutputs/TankDrive/CommandedVolts`. Wheel feedback is under `/AdvantageKit/TankDrive/`. Simulation dimensions and field boundaries are estimates.
