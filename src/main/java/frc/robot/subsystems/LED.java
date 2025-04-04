package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LED extends SubsystemBase {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;
  private double startOfStreak, endOfStreak;
  Color dimBlue, dimGreen;

  public LED(int port, int length) {
    m_rainbowFirstPixelHue = 0;
    startOfStreak = 0.0;
    endOfStreak = 0.0;
    m_led = new AddressableLED(port);
    m_ledBuffer = new AddressableLEDBuffer(length);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

    dimBlue = new Color(0, 0, 20);
    dimGreen = new Color(20, 0, 0);
  }

  public void blinkColor(Color color) {
    double timer = System.currentTimeMillis();
    if (timer % 150 <= 75) {
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setLED(i, color);
      }
    } else {
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setLED(i, Color.kBlack);
      }
    }
    m_led.setData(m_ledBuffer);
  }

  public void rainbow() {
    // For every pixel
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
  }

  public void coralCheck() {
    if (RobotContainer.coralIntake.hasCoral() && RobotContainer.operator.aButton.getAsBoolean()) {
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setLED(i, Color.kRed);
      }
    } else {
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setLED(i, Color.kGreen);
      }
    }
    m_led.setData(m_ledBuffer);
  }

  public Command vorTXStreakCom() {
    return new RunCommand(() -> vorTXStreak(), this);
  }

  public void vorTXStreak() {
    endOfStreak = startOfStreak + m_ledBuffer.getLength() / 2;
    for (int i = (int) startOfStreak; i < (int) endOfStreak; i++) {
      m_ledBuffer.setLED(i % m_ledBuffer.getLength(), dimGreen);
      m_ledBuffer.setLED((i + (m_ledBuffer.getLength() / 2)) % m_ledBuffer.getLength(), dimBlue);
    }
    startOfStreak += 0.25;
    startOfStreak %= m_ledBuffer.getLength();

    m_led.setData(m_ledBuffer);
  }

  public void setColor(Color color) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
    }
    m_led.setData(m_ledBuffer);
  }

  public void visualizeElevatorPosition(double elevatorPosition, double maxElevatorHeight) {
    // Map the elevator position to the LED strip length
    int ledIndex = (int) ((elevatorPosition / maxElevatorHeight) * m_ledBuffer.getLength());

    // Clear the LED strip
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, Color.kBlack);
    }

    // Set the LEDs up to the current position to a specific color
    for (int i = 0; i <= ledIndex; i++) {
      m_ledBuffer.setLED(i, Color.kRed);
    }

    // Flash the LEDs at the top position
    if (elevatorPosition >= maxElevatorHeight) {
      boolean flashState = (System.currentTimeMillis() / 500) % 2 == 0; // Toggle every 500ms
      Color flashColor = flashState ? Color.kYellow : Color.kBlack;
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setLED(i, flashColor);
      }
    }

    // Update the LED data
    m_led.setData(m_ledBuffer);
  }

  public void funny() {
    int r = (int) Math.abs(MathUtil.applyDeadband(RobotContainer.operator.getLeftX(), 0.1) * 255);
    int g = (int) Math.abs(MathUtil.applyDeadband(RobotContainer.operator.getLeftY(), 0.1) * 255);
    int b = (int) Math.abs(MathUtil.applyDeadband(RobotContainer.operator.getRightX(), 0.1) * 255);
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
  }
}
