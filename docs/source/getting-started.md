# Getting started

Open the `16th_Note` repository root (the folder containing `build.gradle`) in WPILib VS Code with the WPILib 2026 Java 17 toolchain. Team number is 9924.

Xbox USB 0: left Y drives, right X turns. RT/LT run intake forward/reverse; RB/LB run indexer forward/reverse. A/X select shooter 50%/75% free speed, B stops it, and holding Y selects shooter speed from the visible hub AprilTag distance. POV Down/Right/Up select pivot +90/+15/-60 degrees. Start with the intake physically home at +90 degrees. CAN 1/2 drive, 3 indexer, 5 pivot, 6 intake, 7 shooter.

The Y-button vision shooter remains stopped until a PhotonVision `HubVisionIO` adapter is installed and three increasing distance/RPS calibration points are entered under `/TunableNumbers/Vision/DistanceShooter`. It does not command the drivetrain, hood, intake, or indexer. While Pete is disabled, set `SaveCalibration` in that group to `true` and confirm the logged save status reads `SAVED` to retain the six values through restart or redeploy.

Run `./gradlew.bat build`, `./gradlew.bat simulateJava`, or `./gradlew.bat deploy`. On Linux/macOS use `./gradlew`.
