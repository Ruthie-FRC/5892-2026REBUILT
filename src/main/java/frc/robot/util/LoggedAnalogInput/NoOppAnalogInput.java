package frc.robot.util.LoggedAnalogInput;

import frc.robot.util.LoggedDIO.DIOInputsAutoLogged;

public class NoOppAnalogInput extends LoggedAnalogInput {
  public NoOppAnalogInput(String name) {
    super(name);
  }

  @Override
  protected void updateInputs(AnalogInputsAutoLogged inputs) {}
}
