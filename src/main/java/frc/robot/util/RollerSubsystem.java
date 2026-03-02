package frc.robot.util;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class RollerSubsystem extends SubsystemBase {
  protected final LoggedTalonFX motor;
  protected final DoubleSupplier forwardVolts;
  protected final DoubleSupplier reverseVolts;
  protected final DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(true);

  public RollerSubsystem(LoggedTalonFX motor, DoubleSupplier forwardVolts) {
    // Use the same speed for both directions
    this(motor, forwardVolts, forwardVolts);
  }

  @Override
  public void periodic() {
    motor.periodic();
  }

  public Command runRoller(DoubleSupplier speed) {
    return runEnd(
        () -> motor.setControl(dutyCycleOut.withOutput(speed.getAsDouble())),
        () -> motor.setControl(dutyCycleOut.withOutput(0)));
  }

  public Command runRoller(Direction direction) {
    DoubleSupplier supplier =
        switch (direction) {
          case FORWARD -> forwardVolts;
          case REVERSE -> reverseVolts;
        };
    return runRoller(supplier);
  }

  public Command stop() {
    return startRun(() -> motor.setControl(dutyCycleOut.withOutput(0)), () -> {});
  }

  public enum Direction {
    FORWARD,
    REVERSE
  }
}
