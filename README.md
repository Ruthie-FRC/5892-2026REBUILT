# 5892 Energy HEROS REBUILT Robot Code
Welcome to the competition robot codebase of team
[5892](https://thebluealliance.com/team/5892/2026/) the [Energy HEROs](https://www.team5892.org/)
for the 2026 [FIRST Robotics Competition](https://www.firstinspires.org/programs/frc/) game [REBUILT](https://youtu.be/_fybREErgyM).

## Resources
* Team 5892 is a member of the Open Alliance. Check out our [build blog](https://www.chiefdelphi.com/t/energy-heros-5892-build-blog-2026-open-alliance/509651).
* Our internal bringup process can be found in [BRINGUP.md](./BRINGUP.md).

## Structure
Our codebase uses the [AdvantageKit](https://docs.advantagekit.org/getting-started/what-is-advantagekit/)
TalonFX [swerve](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template)
and [vision](https://docs.advantagekit.org/getting-started/template-projects/vision-template) templates.
We additionally use self-developed LoggedX APIs to integrate AdvantageKit without creating
boilerplate IO interfaces. Many of these APIs and TunableX APIs add additional tuning capabilities a
using slightly modified versions of 6328 Mechanical Advantage's [LoggedTunableNumber](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/util/LoggedTunableNumber.java).
