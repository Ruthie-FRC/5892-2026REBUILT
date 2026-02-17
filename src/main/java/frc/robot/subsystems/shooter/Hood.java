package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.util.FieldConstants.LinesHorizontal;
import frc.robot.util.FieldConstants.LinesVertical;
import frc.robot.util.LoggedDIO.LoggedDIO;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import frc.robot.util.LoggedTunableMeasure;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MechanismUtil;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Hood extends SubsystemBase {
  /* Hardware */
  private final LoggedTalonFX motor;
  private final LoggedDIO reverseLimit;
  private final LoggedDIO forwardLimit;

  /* Movement Constants */
  private final LoggedTunableMeasure<MutAngle> stowPosition =
      new LoggedTunableMeasure<>("Hood/StowAngle", Degrees.mutable(15));
  public static LoggedTunableNumber stowTrenchGapOffset =
      new LoggedTunableNumber("Hood/stowTrenchGapOffset", 0, "m");
  private final LoggedTunableMeasure<MutAngle> tolerance =
      new LoggedTunableMeasure<>("Hood/Tolerance", Degrees.mutable(5));
  /* Homing */
  private final LoggedTunableNumber homingVoltage =
      new LoggedTunableNumber("Hood/Homing/Voltage", 4, "v");
  private final LoggedTunableNumber homingConfirmationVoltage =
      new LoggedTunableNumber("Hood/Homing/ConfirmVoltage", 4, "v");
  private final LoggedTunableMeasure<MutAngle> homingSwitchPosition =
      new LoggedTunableMeasure<>("Hood/Homing/homePosition", Rotations.mutable(0));
  private final LoggedTunableMeasure<MutAngle> homingConfirmPosition =
      new LoggedTunableMeasure<>("Hood/Homing/homePosition", Rotations.mutable(0.1));

  /* State */
  @AutoLogOutput private Rotation2d targetPosition = Rotation2d.kZero;
  @AutoLogOutput private boolean positionControl = false;
  @AutoLogOutput @Setter private boolean homed = false;
  @AutoLogOutput @Getter private boolean atSetpoint = false;
  @AutoLogOutput private Rectangle2d[] trenchAreas = new Rectangle2d[4];

  /* Control  Requests*/
  private final NeutralOut neutralControl = new NeutralOut();
  private final MotionMagicVoltage mmControl = new MotionMagicVoltage(targetPosition.getMeasure());

  public Hood(LoggedTalonFX motor, LoggedDIO reverseLimit, LoggedDIO forwardLimit) {
    this.motor = motor;
    this.reverseLimit = reverseLimit;
    this.forwardLimit = forwardLimit;
    updateTrenchAreas();
    var config =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(0).withKI(0).withKD(0).withKS(0).withKV(0))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(15)
                    .withMotionMagicAcceleration(30))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(5))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(15));
    motor.withConfig(config).withMMPIDTuning(SlotConfigs.from(config.Slot0), config.MotionMagic);
    setDefaultCommand(aimCommand());
    new Trigger(this::shouldStow).whileTrue(stowCommand());

    SmartDashboard.putData("Hood/SetHomed", runOnce(() -> setHomed(true)).ignoringDisable(true));
  }

  public Command aimCommand() {
    return run(
        () -> {
          if (homed) {
            this.requestPosition(ShotCalculator.getInstance().calculateShot().hoodAngle());
          }
        });
  }

  public Command stowCommand() {
    return startEnd(
            () -> {
              this.requestPosition(new Rotation2d(stowPosition.get()));
            },
            () -> {})
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command homingCommand() {
    return MechanismUtil.buildHomingCommand(
            motor,
            reverseLimit,
            this,
            homingVoltage,
            false,
            homingSwitchPosition::get,
            homingConfirmationVoltage,
            homingConfirmPosition::get)
        .beforeStarting(() -> positionControl = false)
        .finallyDo(
            (i) -> {
              if (!i) {
                setHomed(true);
              }
            });
  }

  /**
   * A command that requests the turret to move to a position. The command completes imminently,
   * without waiting for a tolerance to be achieved.
   *
   * @param position a supplier of the target position
   * @return the command
   */
  public Command requestPosition(Supplier<Rotation2d> position) {
    return runOnce(
        () -> {
          requestPosition(position.get());
        });
  }

  public void requestPosition(Rotation2d position) {
    targetPosition = position;
    positionControl = true;
    setControl();
  }

  /**
   * A command that commands the Turret to move to a position. This command ends when the setpoint
   * is archived
   *
   * @param position a supplier of the target position
   * @return the command
   */
  public Command gotoPosition(Supplier<Rotation2d> position) {
    // I really shouldn't but by creating a functional command I don't create 5 extra objects by
    // separating this out.
    return new FunctionalCommand(
        () -> requestPosition(position.get()),
        () -> {}, // Nothing to do periodically. Motion is controlled in the periodic function
        (i) -> {},
        () -> atSetpoint,
        this);
  }

  @AutoLogOutput(key = "Hood/ShouldStow")
  public boolean shouldStow() {
    final Pose2d pose = RobotState.getInstance().getRobotPosition();
    for (int i = 0; i < trenchAreas.length; i++) {
      if (trenchAreas[i].contains(pose.getTranslation())) {
        return true;
      }
    }
    return false;
  }

  private void updateTrenchAreas() {
    double offset = stowTrenchGapOffset.get();
    trenchAreas =
        new Rectangle2d[] {
          new Rectangle2d(
              new Translation2d(
                  LinesVertical.starting - offset, LinesHorizontal.leftTrenchOpenStart),
              new Translation2d(
                  LinesVertical.neutralZoneNear + offset, LinesHorizontal.leftTrenchOpenEnd)),
          new Rectangle2d(
              new Translation2d(
                  (2 * LinesVertical.center) - LinesVertical.starting - offset,
                  LinesHorizontal.leftTrenchOpenStart),
              new Translation2d(
                  LinesVertical.neutralZoneFar + offset, LinesHorizontal.leftTrenchOpenEnd)),
          new Rectangle2d(
              new Translation2d(
                  LinesVertical.starting - offset, LinesHorizontal.rightTrenchOpenStart),
              new Translation2d(
                  LinesVertical.neutralZoneNear + offset, LinesHorizontal.rightTrenchOpenEnd)),
          new Rectangle2d(
              new Translation2d(
                  (2 * LinesVertical.center) - LinesVertical.starting - offset,
                  LinesHorizontal.rightTrenchOpenStart),
              new Translation2d(
                  LinesVertical.neutralZoneFar + offset, LinesHorizontal.rightTrenchOpenEnd))
        };
  }

  @Override
  public final void periodic() {
    motor.periodic();
    reverseLimit.periodic();
    forwardLimit.periodic();
    atSetpoint = motor.atSetpoint(targetPosition.getMeasure(), tolerance.get());

    ShotCalculator.getInstance().clearCache();
    LoggedTunableNumber.ifChanged(this, (value) -> this.updateTrenchAreas(), stowTrenchGapOffset);

    setControl();
  }

  private void setControl() {
    if (positionControl) {
      motor.setControl(
          mmControl
              .withPosition(targetPosition.getMeasure())
              .withLimitReverseMotion(reverseLimit.get())
              .withLimitForwardMotion(forwardLimit.get()));
    } else {
      motor.setControl(neutralControl);
    }
  }
}
