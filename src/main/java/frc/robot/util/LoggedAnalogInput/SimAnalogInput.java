package frc.robot.util.LoggedAnalogInput;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class SimAnalogInput extends LoggedAnalogInput {

  private final DoubleSupplier simValue;

  public SimAnalogInput(String name, DoubleSupplier simValue) {
    super(name);
    this.simValue = simValue;
  }

  @Override
  protected void updateInputs(AnalogInputsAutoLogged inputs) {
    inputs.value = simValue.getAsDouble();
  }

  public static SimAnalogInput fromNT(String name) {
    LoggedNetworkNumber networkValue = new LoggedNetworkNumber("/Tuning/simInput/dio" + name, 0);
    return new SimAnalogInput(name, networkValue::get);
  }

  /**
   * The function is currently NOOP, returning itself for method chaining. It may be used in the
   * future when noise is simulated.
   *
   * @param bits The number of bits to average over. Higher bits means more averaging, which can
   *     reduce noise but also reduces responsiveness.
   * @return this for method chaining
   */
  @Override
  public LoggedAnalogInput withAverageBits(int bits) {
    return this;
  }
}
