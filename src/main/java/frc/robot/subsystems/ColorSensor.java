// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//WPILIB libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

//REV libraries
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

//Robot imports
import frc.robot.Constants.colorSensConstants;;

public class ColorSensor extends SubsystemBase {
  //REV object declarations
  public final ColorSensorV3 m_colorSensor;
  public final ColorMatch m_colorMatcher;
  public static ColorMatchResult match;

  //Local variables
  public static Color detectedColor;
  public static String colorString = "Unknown";
  public ColorSensor() {
   //SetColor objects
   m_colorSensor = new ColorSensorV3(colorSensConstants.i2cPort);
   m_colorMatcher = new ColorMatch();
   
   //Assigns colors to matcher
   m_colorMatcher.addColorMatch(Color.kBlue);
   m_colorMatcher.addColorMatch(Color.kRed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
