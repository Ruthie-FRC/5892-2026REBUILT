package frc.robot.util.LoggedAnalogInput;

public class NoOppAnalogInput extends LoggedAnalogInput {
  public NoOppAnalogInput(String name) {
    super(name);
  }

  @Override
  protected void updateInputs(AnalogInputsAutoLogged inputs) {}
}
