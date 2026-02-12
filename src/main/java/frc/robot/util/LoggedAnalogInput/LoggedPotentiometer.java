package frc.robot.util.LoggedAnalogInput;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Class for reading analog potentiometers. Analog potentiometers read in an analog voltage that
 * corresponds to a position. The position is in whichever units you choose, by way of the scaling
 * and offset constants passed to the constructor.
 *
 * <p>This class is a copy of wpilib's {@link edu.wpi.first.wpilibj.AnalogPotentiometer}
 */
public class LoggedPotentiometer {
  private LoggedAnalogInput m_LoggedAnalogInput;
  private double m_fullRange;
  private double m_offset;

  /**
   * LoggedPotentiometer constructor.
   *
   * <p>Use the fullRange and offset values so that the output produces meaningful values. I.E: you
   * have a 270 degree potentiometer, and you want the output to be degrees with the halfway point
   * as 0 degrees. The fullRange value is 270.0(degrees) and the offset is -135.0 since the halfway
   * point after scaling is 135 degrees. This will calculate the result from the fullRange times the
   * fraction of the supply voltage, plus the offset.
   *
   * @param input The {@link LoggedAnalogInput} this potentiometer is plugged into.
   * @param fullRange The angular value (in desired units) representing the full 0-5V range of the
   *     input.
   * @param offset The angular value (in desired units) representing the angular output at 0V.
   */
  @SuppressWarnings("this-escape")
  public LoggedPotentiometer(final LoggedAnalogInput input, double fullRange, double offset) {
    m_LoggedAnalogInput = input;

    m_fullRange = fullRange;
    m_offset = offset;
  }

  /**
   * LoggedPotentiometer constructor.
   *
   * <p>Use the fullRange and offset values so that the output produces meaningful values. I.E: you
   * have a 270 degree potentiometer, and you want the output to be degrees with the starting point
   * as 0 degrees. The scale value is 270.0(degrees).
   *
   * @param input The {@link LoggedAnalogInput} this potentiometer is plugged into.
   * @param scale The scaling to multiply the voltage by to get a meaningful unit.
   */
  public LoggedPotentiometer(final LoggedAnalogInput input, double scale) {
    this(input, scale, 0);
  }

  /**
   * LoggedPotentiometer constructor.
   *
   * <p>The potentiometer will return a value between 0 and 1.0.
   *
   * @param input The {@link LoggedAnalogInput} this potentiometer is plugged into.
   */
  public LoggedPotentiometer(final LoggedAnalogInput input) {
    this(input, 1, 0);
  }

  /**
   * Get the current reading of the potentiometer.
   *
   * @return The current position of the potentiometer.
   */
  public double get() {
    if (m_LoggedAnalogInput == null) {
      return m_offset;
    }
    return (m_LoggedAnalogInput.get() / RobotController.getVoltage5V()) * m_fullRange + m_offset;
  }
}
