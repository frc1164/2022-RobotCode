// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//WPI imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

//REV imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

//Robot imports
import frc.robot.Constants.motorConstants;
import frc.robot.Constants.colorSensorConstants;

public class Shooter extends SubsystemBase {
  public CANSparkMax shootMot1,shootMot2, indexMot;
  public RelativeEncoder shootEnc1, shootEnc2;
  public final ColorSensorV3 m_colorSensor;
  public final ColorMatch m_colorMatcher;

  public Shooter() {
    //Shooter Motors
    shootMot1 = new CANSparkMax(motorConstants.SPEED_CONT16, MotorType.kBrushless);
    shootMot2 = new CANSparkMax(motorConstants.SPEED_CONT17, MotorType.kBrushless);

    shootMot1.setIdleMode(IdleMode.kCoast);
    shootMot2.setIdleMode(IdleMode.kCoast);

    shootMot1.setInverted(motorConstants.SHOOTMOT1_INVERT);

    //Shooter Encoders
    shootEnc1 = shootMot1.getEncoder();
    shootEnc2 = shootMot2.getEncoder();

    //Color Sensor
    m_colorSensor = new ColorSensorV3(colorSensorConstants.i2cPort);
    m_colorMatcher = new ColorMatch();

    //Smartdashboard
    SmartDashboard.putNumber("Shooter Motor Velocity Input", shootEnc1.getVelocity());

    //Indexer Motor
    indexMot = new CANSparkMax(motorConstants.SPEED_CONT18, MotorType.kBrushless);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIndex (double speed) {
    indexMot.set(speed);
  }

  public void runShooter () {
    //Get values from Smartdashboard
    double speed = SmartDashboard.getNumber("Shooter Motor Velocity Input", 0.0);

    //Set Motor Speeds
    shootMot1.set(speed);
    shootMot2.set(speed);

    SmartDashboard.putNumber("Motor 1 Velocity", shootEnc1.getVelocity());
    SmartDashboard.putNumber("Motor 2 Velocity", shootEnc2.getVelocity());
  }

  public void stopShooter () {
    shootMot1.set(0.0);
    shootMot2.set(0.0);
  }

  public String readBall () {
    Color detectedColor = m_colorSensor.getColor();


    /**
     * The sensor returns a raw IR value of the infrared light detected.
     */
    double IR = m_colorSensor.getIR();
    double red = detectedColor.red;
    double blue = detectedColor.blue;

    if (IR > 15) {
      SmartDashboard.putBoolean("has_ball", true);
    }
    else SmartDashboard.putBoolean("has_ball", false);

    if (red >= 0.4 && red <= 0.6 && blue <= 0.2) {
      return "RED";
    }

    else if (blue >= 0.33 && blue <= 0.5 && red <= 0.23) {
      return "BLUE";
    }

    else return "NONE";

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     *
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putString("raw", Integer.toHexString(detectedColor.hashCode()));

    */
  }


}
