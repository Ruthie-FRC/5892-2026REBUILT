# Robot Bringup Process
## 0: General
1. Flash the roboRIO with the latest image
2. After the robot is powered on, name, set IDs, license, and firmware update all devices according to TBD
3. Set every subsystem to NoOpp. in `RobotContainer`. Re-enable subsystems one at a time as they are brought up.
## 1: Swerve
1. Follow the [TalonFX Swerve Template](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template) to configure the project in Phoenix Tuner and test the swerve modules
3. When time allows, tune the drivetrain according to the same guide. Note: PIDFF values can be set in AdvantageScope
## 2: Vision
1. Ensure all coprocessors are up to date and connected.
2. Identify and calibrate each camera using the ChArUCo board.
3. Setup multi-tag pose estimation on every coprocessor.
4. Measure the offset of each camera from the center of the robot on the carpet and add to code. Reference the [WPILIB Coordinate System](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html).
5. Log each offset (as an output) via a `static {}` method. Simulate and view these offsets on an AdvantageScope 3d field, where the field and each camera offset is displayed as an axis. Ensure the offsets are correct and adjust as needed.
## 3: Turret
1. Ensure the auto-aiming commands are disabled. If needed, also temporarily switch all turret limit switches to sim/NT controlled
2. View the Absolute turret position on in AdvantageScope. Ensure it is moving in the correct direction along the entire length of the turret's rotation.
3. Ensure the motor encoder is following the absolute position in the correct direction.
4. Move the turret to be facing forward (0 deg). Run the SetHomed command from the dashboard.
5. Move the turret to its limits, making sure the position is correct in AdvantageScope. Pay special attention to 0, -120, and 120 deg which are the PID tuning poses.
6. Tune the turret's PID values via the dashboard until it can reliably turn to the 0, -120, and 120 deg using A, X, and B on the test controller, respectively. Persist the values to robot code when finished.
7. Redeploy code with autoAim.
## 4: Hood
1. Ensure the auto-aiming commands are disabled. If needed, also temporarily switch hood limit switches to sim/NT controlled
2. Move the hood to be the bottom. Run the SetHomed command from the dashboard.
3. Move the hood to its limits, making sure the position is correct in AdvantageScope. Pay special attention to 0, -120, and 120 deg which are the PID tuning poses.
4. Tune the turret's PID values via the dashboard until it can reliably turn to the 0, -120, and 120 deg using A, X, and B on the test controller, respectively. Persist the values to robot code when finished.
5. Redeploy code with autoAim.
