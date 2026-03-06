package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class Kicker extends SubsystemBase {
  private final DoubleSupplier forwardSpeed =
      new LoggedTunableNumber("Kicker/SpeedRotPerSec", 70.00);
  private final DoubleSupplier unjamSpeed = new LoggedTunableNumber("Kicker/UnJamDutyCycle", -1.00);

  private final LoggedTalonFX motor;
  private final VelocityDutyCycle out = new VelocityDutyCycle(forwardSpeed.getAsDouble());
  private final NeutralOut neutralOut = new NeutralOut();
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(unjamSpeed.getAsDouble());

  public Kicker(LoggedTalonFX motor) {
    var config =
        LoggedTalonFX.buildStandardConfig(40, 40)
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.06)
                    .withKI(0)
                    .withKD(0)
                    .withKS(0.023)
                    .withKV(0.008)
                    .withKA(0))
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    this.motor = motor.withConfig(config).withPIDTunable(config.Slot0);
    this.setDefaultCommand(this.runRoller());
  }

  @Override
  public void periodic() {
    motor.periodic();
  }

  public Command runRoller() {
    return runEnd(
        () -> motor.setControl(out.withVelocity(forwardSpeed.getAsDouble())),
        () -> motor.setControl(neutralOut));
  }

  public Command unjam() {
    return runEnd(
        () -> motor.setControl(dutyCycleOut.withOutput(unjamSpeed.getAsDouble())),
        () -> motor.setControl(neutralOut));
  }

  public Command stop() {
    return startRun(() -> motor.setControl(neutralOut), () -> {});
  }

  public enum Direction {
    FORWARD,
    REVERSE
  }
}
