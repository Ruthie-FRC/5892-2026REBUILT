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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil;
import frc.robot.RobotState;

public class Led extends SubsystemBase {
  private final LoggedTunableNumber ledStartIndex = new LoggedTunableNumber("LED/StartIndex", 0);
  private final LoggedTunableNumber ledEndIndex = new LoggedTunableNumber("LED/EndIndex", 1);
  public final CANdle candle = new CANdle(40);

  private final SolidColor solidColor = new SolidColor(
    (int) Math.round(ledStartIndex.get()),
    (int) Math.round(ledEndIndex.get()));

  private final RGBWColor redColor = new RGBWColor(255, 0, 0);
  private final RGBWColor orangeColor = new RGBWColor(255, 153, 28);
  private final RGBWColor blackColor = new RGBWColor(0, 0, 0);
  private final RGBWColor whiteColor = new RGBWColor(255, 255, 255);
  /** Creates a new Led. */
  public Led() {
    CANdleConfiguration candleConfiguration =
        new CANdleConfiguration()
            .withLED(
                new LEDConfigs()
                    .withBrightnessScalar(1)
                    .withStripType(StripTypeValue.GRB)
                    .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning));
    PhoenixUtil.tryUntilOk(5, () -> candle.getConfigurator().apply(candleConfiguration));
  }

  public void setColor(RGBWColor newColor) {
    candle.setControl(
        solidColor.withColor(newColor));
  }

  public void setFlashing(int startIndex, int endIndex) {
    candle.setControl(new StrobeAnimation(startIndex, endIndex));
  }

  public Command flashTwoColors(RGBWColor c1, RGBWColor c2) {
    return Commands.sequence(
            Commands.runOnce(() -> setColor(c1), this),
            Commands.waitSeconds(0.2),
            Commands.runOnce(() -> setColor(c2), this),
            Commands.waitSeconds(0.2))
        .repeatedly();
  }

  @Override
  public void periodic() {
    if (RobotState.getInstance().shouldStow) {
      flashTwoColors(redColor, blackColor);
    } else if(RobotState.getInstance().isTurretNearWrapPoint()) {
      flashTwoColors(orangeColor, whiteColor);
    } else if(RobotState.getInstance().isTurretAtSetpoint()) {
      setColor(orangeColor);
    } else{
      setColor(whiteColor);
    }
  }
}
