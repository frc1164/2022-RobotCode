// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LED extends SubsystemBase {
  /** Creates a new LED. */
private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(48);
private AddressableLED m_led = new AddressableLED(LEDConstants.LEDport);
  public LED() {
    rainbow(0, 48, 0, 0);
    LED_init();
  }

  private void rainbow(int startPos, int Length, double rainbowOffset, double hueModdifier) {
    // if (startPos + Length < m_ledBuffer.getLength()) {
      for (var i = startPos; i < startPos + Length; i++) {
        final int hue = (int) (((((rainbowOffset + i) * 180) / Length) + hueModdifier) % 180);
        m_ledBuffer.setHSV(i, (hue), 255, 128);
      }
    }

  public void LED_init() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
