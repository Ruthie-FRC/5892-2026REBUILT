package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.FieldConstants.LinesHorizontal;
import frc.robot.util.FieldConstants.LinesVertical;
import frc.robot.util.LoggedTalon.TalonFXS.LoggedTalonFXS;
import frc.robot.util.LoggedTunableMeasure;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  /* Hardware */
  private final LoggedTalonFXS motor;

  /* Movement Constants */
  private final LoggedTunableMeasure<MutAngle> downPosition =
      new LoggedTunableMeasure<>("Hood/DownPosition", Degrees.mutable(18.575));
  private final LoggedTunableMeasure<MutAngle> stowPosition =
      new LoggedTunableMeasure<>("Hood/StowAngle", Degrees.mutable(15));
  // Hard limits for the hood angle (relative to vertical). Requests will be clamped into this
  // range to prevent commanding into physical stops.
  private final LoggedTunableMeasure<MutAngle> minAngle =
      new LoggedTunableMeasure<>("Hood/MinAngle", Degrees.mutable(0));
  private final LoggedTunableMeasure<MutAngle> maxAngle =
      new LoggedTunableMeasure<>("Hood/MaxAngle", Degrees.mutable(45));
  // Slow-stop zone: within this angular distance (deg) from either limit the hood will be
  // slowed to avoid hitting physical stops at full speed.
  private final LoggedTunableMeasure<MutAngle> slowZone =
      new LoggedTunableMeasure<>("Hood/SlowZone", Degrees.mutable(6));
  // Maximum duty (absolute) permitted while in the slow zone when approaching a limit.
  private final LoggedTunableNumber slowStopMaxDuty =
      new LoggedTunableNumber("Hood/SlowStopMaxDuty", 0.25, "%");
  public static LoggedTunableNumber stowTrenchGapOffset =
      new LoggedTunableNumber("Hood/stowTrenchGapOffset", 0, "m");
  private final LoggedTunableMeasure<MutAngle> tolerance =
      new LoggedTunableMeasure<>("Hood/Tolerance", Degrees.mutable(2));
  /* Homing */
  private final LoggedTunableNumber homingDutyCycle =
      new LoggedTunableNumber("Hood/Homing/DutyCycle", -0.1, "%");
  private final LoggedTunableNumber homingCurrentThreshold =
      new LoggedTunableNumber("Hood/Homing/CurrentThreshold", 10, "A");

  /* State */
  /** The target position of the motor. 0 is the hood resting on the turret. */
  @AutoLogOutput private final MutAngle targetPosition = Degrees.mutable(0);

  @AutoLogOutput private boolean positionControl = false;
  @AutoLogOutput @Setter private boolean homed = true;
  @AutoLogOutput @Getter private boolean atSetpoint = false;
  @AutoLogOutput private Rectangle2d[] trenchAreas = new Rectangle2d[4];

  /* Control  Requests*/
  private final NeutralOut neutralControl = new NeutralOut();
  private final MotionMagicDutyCycle mmControl = new MotionMagicDutyCycle(targetPosition);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  // The last requested duty-cycle from high-level code. Used so periodic soft-stop can
  // override open-loop commands (like manual duty tests or homing) when approaching limits.
  private double lastRequestedDuty = 0.0;

  public Hood(LoggedTalonFXS motor) {
    this.motor = motor;
    updateTrenchAreas();
    var config =
        new TalonFXSConfiguration()
            .withCommutation(
                new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST))
            .withSlot0(new Slot0Configs().withKP(14).withKI(0).withKD(0).withKS(0.017).withKV(0.2))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(15)
                    .withMotionMagicAcceleration(30))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(15))
            .withExternalFeedback(new ExternalFeedbackConfigs().withSensorToMechanismRatio(36));
    motor.withConfig(config).withMMPIDTuning(SlotConfigs.from(config.Slot0), config.MotionMagic);
    // 12t pulley -> 24t, 10t gear -> 180t spur gear
    // 2:1 * 18:1 = 36:1 overall

    // TODO: Uncomment before flight
    setDefaultCommand(aimCommand());
    // new Trigger(this::shouldStow).whileTrue(stowCommand());

    SmartDashboard.putData("Hood/SetHomed", runOnce(() -> setHomed(true)).ignoringDisable(true));
  }

  public Command aimCommand() {
    return run(
        () -> {
          if (homed) {
            this.requestAngle(ShotCalculator.getInstance().calculateShot().hoodAngle());
          }
        });
  }

  public Command stowCommand() {
    return startEnd(
            () -> {
              this.requestAngle(new Rotation2d(stowPosition.get()));
            },
            () -> {})
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  // TODO: Create a real homing sequence w/ sensorless homing
  public Command homingCommand() {
    return new FunctionalCommand(
        () -> {
          this.positionControl = false;
          lastRequestedDuty = homingDutyCycle.get();
          motor.setControl(dutyCycleOut.withOutput(homingDutyCycle.get()));
        },
        () -> {},
        (i) -> {
          lastRequestedDuty = 0.0;
          motor.setControl(dutyCycleOut.withOutput(0));
          if (!i) {
            setHomed(true);
            motor.setPosition(Degrees.of(0));
          }
        },
        () -> Math.abs(motor.getPrimaryTorqueCurrentAmps()) >= homingCurrentThreshold.get(),
        this);
  }

  /**
   * A command that requests the turret to move to a robot-relative angle. The command completes
   * imminently, without waiting for a tolerance to be achieved.
   *
   * @param angle a supplier of the target angle. The angle is relative to vertical. 0 is vertical,
   *     90 is horizontal
   * @return the command
   */
  public Command requestAngle(Supplier<Rotation2d> angle) {
    return runOnce(
        () -> {
          requestAngle(angle.get());
        });
  }

  /**
   * @param angle the target angle relative to vertical. 0 is vertical, 90 is horizontal
   */
  public void requestAngle(Rotation2d angle) {
    Logger.recordOutput("Hood/RequestedAngle", angle.getDegrees(), "deg");
    // Clamp the requested angle to the allowed range so we never command beyond physical
    // limits. The LoggedTunableMeasures use angular measures, so compare in radians via
    // baseUnitMagnitude().
    double minAng = minAngle.get().baseUnitMagnitude();
    double maxAng = maxAngle.get().baseUnitMagnitude();
    double req = angle.getRadians();
    if (req < minAng) {
      req = minAng;
    } else if (req > maxAng) {
      req = maxAng;
    }
    final Rotation2d clamped = new Rotation2d(req);
    angleToPosition(clamped, targetPosition);
    // We're commanding a position-based move
    this.positionControl = true;
    motor.setControl(mmControl.withPosition(targetPosition));
  }

  /**
   * A command that commands the Turret to move to a angle. This command ends when the setpoint is
   * archived
   *
   * @param angle a supplier of the target angle. The angle is relative to vertical. 0 is vertical,
   *     90 is horizontal
   * @return the command
   */
  public Command gotoAngle(Supplier<Rotation2d> angle) {
    // I really shouldn't but by creating a functional command I don't create 5 extra objects by
    // separating this out.
    return new FunctionalCommand(
        () -> requestAngle(angle.get()),
        () -> {}, // Nothing to do periodically. Motion is controlled in the periodic function
        (i) -> {},
        () -> atSetpoint,
        this);
  }

  public boolean shouldStow() {
    boolean shouldStow = false;
    final Pose2d pose = RobotState.getInstance().getRobotPosition();
    for (int i = 0; i < trenchAreas.length; i++) {
      if (trenchAreas[i].contains(pose.getTranslation())) {
        shouldStow = true;
        break;
      }
    }
    RobotState.getInstance().setShouldStow(shouldStow);
    Logger.recordOutput("Hood/ShouldStow", shouldStow);
    return shouldStow;
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
    atSetpoint = motor.atSetpoint(targetPosition, tolerance.get());
    Logger.recordOutput("Hood/Angle", positionToAngle(motor.getPosition()).getDegrees(), "deg");

    ShotCalculator.getInstance().clearCache();
    LoggedTunableNumber.ifChanged(this, (value) -> this.updateTrenchAreas(), stowTrenchGapOffset);
    // Soft/slow-stop behavior: if we're performing position control and are approaching
    // a hard limit, override the motor control with a scaled duty cycle so the hood doesn't
    // slam into the physical stop at full speed.
    try {
      // Current angle (robot-relative)
      Rotation2d currentAngle = positionToAngle(motor.getPosition());
      double current = currentAngle.getRadians();

      double minA = minAngle.get().baseUnitMagnitude();
      double maxA = maxAngle.get().baseUnitMagnitude();
      double slowZoneRad = slowZone.get().baseUnitMagnitude();
      double distToMin = current - minA;
      double distToMax = maxA - current;

      // Velocity and requested duty help us decide if we're moving/commanded toward a limit.
      double vel = motor.getVelocity().baseUnitMagnitude();
      double signVel = Math.signum(vel);
      double signReq = Math.signum(lastRequestedDuty);

      // Decide the primary movement direction: prefer velocity if non-zero, otherwise fall back
      // to the last requested duty.
      double signMove = 0.0;
      final double VEL_EPS = 1e-6;
      if (Math.abs(vel) > VEL_EPS) {
        signMove = signVel;
      } else {
        signMove = signReq;
      }

      // We intervene when:
      // - positionControl is active (we're commanding a Motion Magic move), OR
      // - we have an open-loop duty request moving toward the same limit, OR
      // - the motor is currently moving toward a limit (by velocity) and is inside slow zone.
      if (positionControl
          || (lastRequestedDuty != 0.0 && Math.signum(lastRequestedDuty) == signMove)
          || (Math.abs(vel) > VEL_EPS && signMove != 0.0)) {
        // Approaching min limit
        if (signMove < 0 && distToMin <= slowZoneRad && distToMin >= 0) {
          double frac = Math.max(0.0, distToMin / slowZoneRad);
          double maxDuty = Math.abs(slowStopMaxDuty.get());
          double scaledMag = maxDuty * Math.max(0.15, frac);
          double scaled = -Math.abs(scaledMag);
          motor.setControl(dutyCycleOut.withOutput(scaled));
        }
        // Approaching max limit
        else if (signMove > 0 && distToMax <= slowZoneRad && distToMax >= 0) {
          double frac = Math.max(0.0, distToMax / slowZoneRad);
          double maxDuty = Math.abs(slowStopMaxDuty.get());
          double scaledMag = maxDuty * Math.max(0.15, frac);
          double scaled = Math.abs(scaledMag);
          motor.setControl(dutyCycleOut.withOutput(scaled));
        }
      }
    } catch (Exception e) {
      // Defensive: don't let soft-stop logic crash periodic
      Logger.recordOutput("Hood/SoftStopError", e.toString());
    }
  }

  /**
   * Converts the robot relative angle to motor position. This is just a helper; would be static if
   * it didn't use tuned values.
   *
   * <p>This is a c style function for them GC savings.
   *
   * @param angle the angle to convert, relative to vertical down.
   * @param positionOut a reference to the position to write to. This is mutated rather than
   *     returned to save on GC.
   */
  private void angleToPosition(Rotation2d angle, MutAngle positionOut) {
    positionOut.mut_setBaseUnitMagnitude(
        angle.getRadians() - downPosition.get().baseUnitMagnitude());
  }

  /**
   * Converts the motor position to a robot relative angle. This is just a helper; would be static
   * if it didn't use tuned values.
   *
   * @param position the position to convert
   * @return the angle, relative to vertical down
   */
  private Rotation2d positionToAngle(Angle position) {
    return new Rotation2d(position.baseUnitMagnitude() + downPosition.get().baseUnitMagnitude());
  }

  /**
   * A simple test command that runs the motor at a given duty cycle. Useful for testing during
   * bringup.
   *
   * @param dutyCycle the duty cycle to run the motor at, from -1 to 1
   * @return the command
   */
  public Command dutyCycleTestCommand(double dutyCycle) {
    return startEnd(
        () -> {
          this.positionControl = false;
          lastRequestedDuty = dutyCycle;
          motor.setControl(dutyCycleOut.withOutput(dutyCycle));
        },
        () -> {
          lastRequestedDuty = 0.0;
          motor.setControl(dutyCycleOut.withOutput(0));
        });
  }
}
