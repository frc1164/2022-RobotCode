// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//WPI imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DigitalInput;

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
import frc.robot.Constants.limitSwitchConstants;

public class Shooter extends SubsystemBase {
  public CANSparkMax shootMot1,shootMot2; 
  public static CANSparkMax feederMot, liftMot;
  public RelativeEncoder shootEnc1, shootEnc2;
  public final ColorSensorV3 m_colorSensor;
  public final ColorMatch m_colorMatcher;
  public static RelativeEncoder liftEnc;
  public static DigitalInput topLimitSwitch, botLimitSwitch;

  public Shooter() {
    //Shooter Motors
    shootMot1 = new CANSparkMax(motorConstants.SPEED_CONT16, MotorType.kBrushless);
    shootMot2 = new CANSparkMax(motorConstants.SPEED_CONT17, MotorType.kBrushless);
    feederMot = new CANSparkMax(motorConstants.SPEED_CONT18, MotorType.kBrushless);
    liftMot = new CANSparkMax(motorConstants.SHOOTER_LIFT, MotorType.kBrushless);

    shootMot1.setIdleMode(IdleMode.kCoast);
    shootMot2.setIdleMode(IdleMode.kCoast);
    feederMot.setIdleMode(IdleMode.kCoast);
    liftMot.setIdleMode(IdleMode.kCoast);

    shootMot1.setInverted(motorConstants.SHOOTMOT1_INVERT);

    //Shooter Encoders
    shootEnc1 = shootMot1.getEncoder();
    shootEnc2 = shootMot2.getEncoder();
    liftEnc = liftMot.getEncoder();

    //Color Sensor
    m_colorSensor = new ColorSensorV3(colorSensorConstants.i2cPort);
    m_colorMatcher = new ColorMatch();

    //Smartdashboard
    SmartDashboard.putNumber("Lift Motor Velocity Input", 0.0);
    SmartDashboard.putNumber("Shooter Motor Velocity Input", shootEnc1.getVelocity());

    //Lift limitswitches
    botLimitSwitch = new DigitalInput(limitSwitchConstants.BOTTOM_LIMIT_SWITCH_PORT);
    topLimitSwitch = new DigitalInput(limitSwitchConstants.TOP_LIMIT_SWITCH_PORT);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder pos", liftEnc.getPosition());
  }

  public static void runFeeder (double speed) {
    feederMot.set(speed);
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

  public static boolean liftInit() {
    if (botLimitSwitch.get()){
      liftMot.set(0.1);
    }
    if (!botLimitSwitch.get()){
      liftMot.set(0.0);
      liftEnc.setPosition(0);
      return true;
    }
    return false;
  }

  public static void runLift() {
    double speed = SmartDashboard.getNumber("Lift Motor Velocity Input", 0.0);
    if (speed > 0.0){
      if(botLimitSwitch.get()){
        liftMot.set(speed);
      }
      else {
        liftMot.set(0.0);
      }
    }
    else if (speed < 0.0) {
      if (topLimitSwitch.get()){
         liftMot.set(speed);
      }
      else {
        liftMot.set(0.0);
      }
    }
    else {
      liftMot.set(0.0);
    }
    SmartDashboard.putNumber("Lift position", liftEnc.getPosition());
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
