// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.Hood;

public class Led extends SubsystemBase {
  public static CANdle candle = new CANdle(40);

  /** Creates a new Led. */
  public Led() {
    CANdleConfiguration candleConfiguration =
        new CANdleConfiguration()
            .withLED(
                new LEDConfigs()
                    .withBrightnessScalar(1)
                    .withStripType(StripTypeValue.RGBW)
                    .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning));
    candle.getConfigurator().apply(candleConfiguration);
  }

  public void setColor(RGBWColor newColor, int startIndex, int endIndex) {
    candle.setControl(
        new SolidColor(startIndex, endIndex) // Controls what LEDS receive color changes
            .withColor(newColor));
  }

  public void setFlashing(int startIndex, int endIndex) {
    candle.setControl(new StrobeAnimation(startIndex, endIndex));
  }

  public Command flashTwoColors(int startIndex, int endIndex, RGBWColor c1, RGBWColor c2) {
      return Commands.sequence(
          Commands.runOnce(() -> setColor(c1, startIndex, endIndex), this),
          Commands.waitSeconds(0.2),
          Commands.runOnce(() -> setColor(c2, startIndex, endIndex), this),
          Commands.waitSeconds(0.2)
      ).repeatedly();
  }
  /* Start/end indexes currently omitted bc idk what they control lol */
  public void hoodIndicator(){
    if(RobotState.getInstance().shouldStow){
      flashTwoColors(0, 0, new RGBWColor(255, 0, 0, 0), new RGBWColor(0, 0, 0, 0));
    }
  }
  public void lockedIndicator(){ // When turret at setpoint
    if(RobotState.getInstance().turretAtSetpoint){
      setColor(new RGBWColor(255, 153, 28, 0), 0, 0);
    }else{
      setColor(new RGBWColor(255, 255, 255, 255), 0, 0);
    }
  }
  public void wrapIndicator(){
    if(RobotState.getInstance().turretNearWrapPoint){
      flashTwoColors(0, 0, new RGBWColor(255, 153, 28, 0), new RGBWColor(255, 255, 255, 255));
    }
  }

  @Override
  public void periodic() {
    // So the issue I have with this logic is that it will obliviously cycle through booleans
    // This will flash colors a lot, so we might want to make it so that the colors change based on the LAST change from this list of checks. 
    hoodIndicator();
    lockedIndicator();
    wrapIndicator();    
    // This method will be called once per scheduler run
  }
}
