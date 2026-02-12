package frc.robot.util.LoggedAnalogInput;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Class for reading analog potentiometers. Analog potentiometers read in an analog voltage that
 * corresponds to a position. The position is in whichever units you choose, by way of the scaling
 * and offset constants passed to the constructor.
 *
 * <p>This class is a copy of wpilib's {@link edu.wpi.first.wpilibj.AnalogPotentiometer}
 */
public class LoggedPotentiometer {
  private AnalogInput m_analogInput;
  private boolean m_initAnalogInput;
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
   * @param channel The analog input channel this potentiometer is plugged into. 0-3 are on-board
   *     and 4-7 are on the MXP port.
   * @param fullRange The scaling to multiply the fraction by to get a meaningful unit.
   * @param offset The offset to add to the scaled value for controlling the zero value
   */
  @SuppressWarnings("this-escape")
  public LoggedPotentiometer(final int channel, double fullRange, double offset) {
    this(new AnalogInput(channel), fullRange, offset);
    m_initAnalogInput = true;
  }

  /**
   * LoggedPotentiometer constructor.
   *
   * <p>Use the fullRange and offset values so that the output produces meaningful values. I.E: you
   * have a 270 degree potentiometer, and you want the output to be degrees with the halfway point
   * as 0 degrees. The fullRange value is 270.0(degrees) and the offset is -135.0 since the halfway
   * point after scaling is 135 degrees. This will calculate the result from the fullRange times the
   * fraction of the supply voltage, plus the offset.
   *
   * @param input The {@link AnalogInput} this potentiometer is plugged into.
   * @param fullRange The angular value (in desired units) representing the full 0-5V range of the
   *     input.
   * @param offset The angular value (in desired units) representing the angular output at 0V.
   */
  @SuppressWarnings("this-escape")
  public LoggedPotentiometer(final AnalogInput input, double fullRange, double offset) {
    m_analogInput = input;
    m_initAnalogInput = false;

    m_fullRange = fullRange;
    m_offset = offset;
  }

  /**
   * LoggedPotentiometer constructor.
   *
   * <p>Use the scale value so that the output produces meaningful values. I.E: you have a 270
   * degree potentiometer, and you want the output to be degrees with the starting point as 0
   * degrees. The scale value is 270.0(degrees).
   *
   * @param channel The analog input channel this potentiometer is plugged into. 0-3 are on-board
   *     and 4-7 are on the MXP port.
   * @param scale The scaling to multiply the voltage by to get a meaningful unit.
   */
  public LoggedPotentiometer(final int channel, double scale) {
    this(channel, scale, 0);
  }

  /**
   * LoggedPotentiometer constructor.
   *
   * <p>Use the fullRange and offset values so that the output produces meaningful values. I.E: you
   * have a 270 degree potentiometer, and you want the output to be degrees with the starting point
   * as 0 degrees. The scale value is 270.0(degrees).
   *
   * @param input The {@link AnalogInput} this potentiometer is plugged into.
   * @param scale The scaling to multiply the voltage by to get a meaningful unit.
   */
  public LoggedPotentiometer(final AnalogInput input, double scale) {
    this(input, scale, 0);
  }

  /**
   * LoggedPotentiometer constructor.
   *
   * <p>The potentiometer will return a value between 0 and 1.0.
   *
   * @param channel The analog input channel this potentiometer is plugged into. 0-3 are on-board
   *     and 4-7 are on the MXP port.
   */
  public LoggedPotentiometer(final int channel) {
    this(channel, 1, 0);
  }

  /**
   * LoggedPotentiometer constructor.
   *
   * <p>The potentiometer will return a value between 0 and 1.0.
   *
   * @param input The {@link AnalogInput} this potentiometer is plugged into.
   */
  public LoggedPotentiometer(final AnalogInput input) {
    this(input, 1, 0);
  }

  /**
   * Get the current reading of the potentiometer.
   *
   * @return The current position of the potentiometer.
   */
  public double get() {
    if (m_analogInput == null) {
      return m_offset;
    }
    return (m_analogInput.getAverageVoltage() / RobotController.getVoltage5V()) * m_fullRange
        + m_offset;
  }
}
