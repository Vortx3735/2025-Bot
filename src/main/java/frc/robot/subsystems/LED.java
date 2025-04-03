package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LED extends SubsystemBase {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;
  private double startOfStreak, endOfStreak;

  public LED(int port, int length) {
    m_rainbowFirstPixelHue = 0;
    startOfStreak = 0.0;
    endOfStreak = 0.0;
    m_led = new AddressableLED(port);
    m_ledBuffer = new AddressableLEDBuffer(length);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
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
        m_ledBuffer.setLED(i, Color.kAntiqueWhite);
      }
    } else {
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setLED(i, Color.kRed);
      }
    }
    m_led.setData(m_ledBuffer);
  }

  public void vorTXStreak() {
    endOfStreak = startOfStreak + m_ledBuffer.getLength() / 2;
    for (int i = (int) startOfStreak; i < (int) endOfStreak; i++) {
      m_ledBuffer.setLED(i % m_ledBuffer.getLength(), Color.kGreen);
      m_ledBuffer.setLED(
          (i + (m_ledBuffer.getLength() / 2)) % m_ledBuffer.getLength(), Color.kBlue);
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

  // public void funny() {
  //     int r = (int) Math.abs(MathUtil.applyDeadband(RobotContainer.driver.getLeftX(), 0.1)*255);
  //     int g = (int) Math.abs(MathUtil.applyDeadband(RobotContainer.driver.getLeftY(), 0.1)*255);
  //     int b = (int) Math.abs(MathUtil.applyDeadband(RobotContainer.driver.getRightX(), 0.1)*255);
  //     for (int i = 0; i < m_ledBuffer.getLength(); i++) {
  //         m_ledBuffer.setRGB(i, r, g, b);
  //     }
  //     m_led.setData(m_ledBuffer);
  // }
}
