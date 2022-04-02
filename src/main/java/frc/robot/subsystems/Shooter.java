// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//WPI imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
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
import frc.robot.subsystems.Vision;

public class Shooter extends SubsystemBase {
  public CANSparkMax shootMot1,shootMot2; 
  public static CANSparkMax feederMot, liftMot;
  public RelativeEncoder shootEnc1, shootEnc2;
  public final ColorSensorV3 m_colorSensor;
  public final ColorMatch m_colorMatcher;
  public static RelativeEncoder liftEnc;
  public static DigitalInput topLimitSwitch, botLimitSwitch;

  //Shoot lift PID values


  //Declare PID contoller and values
  private ShuffleboardTab tab = Shuffleboard.getTab("PID LL Settings");
  private NetworkTableEntry kP = tab.add("Line P", 0.8).getEntry();
  private NetworkTableEntry kI = tab.add("Line I", 0.016).getEntry();
  private NetworkTableEntry kD = tab.add("Line D", 0.8).getEntry();
  public static double P, I, D, dP, min_Command;
  public static double PIDout, steeringAdjust;
  static PIDController testPID = new PIDController(P, I, D);

  //Decalare driverstation
  DriverStation ds;

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
    liftMot.setInverted(motorConstants.SHOOTER_LIFT_INVERT);

    //Shooter Encoders
    shootEnc1 = shootMot1.getEncoder();
    shootEnc2 = shootMot2.getEncoder();
    liftEnc = liftMot.getEncoder();

    //Color Sensor
    m_colorSensor = new ColorSensorV3(colorSensorConstants.i2cPort);
    m_colorMatcher = new ColorMatch();

    //Driverstation

    //Lift limitswitches
    topLimitSwitch = new DigitalInput(limitSwitchConstants.TOP_LIMIT_SWITCH_PORT);
    botLimitSwitch = new DigitalInput(limitSwitchConstants.BOTTOM_LIMIT_SWITCH_PORT);

    //PID Init
    PIDout = 0.0;
    P = kP.getDouble(0.0);
    I = kI.getDouble(0.0);
    D = kD.getDouble(0.0);
    testPID.setPID(P, I, D);
    testPID.setSetpoint(0.0);
    testPID.enableContinuousInput(5.0, 110.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder pos", liftEnc.getPosition());
    SmartDashboard.putBoolean("Shooter Top Limit", topLimitSwitch.get());
    SmartDashboard.putBoolean("Shooter Bottom Limit", botLimitSwitch.get());

  }

  public void runFeeder (double speed) {
    feederMot.set(speed);
  }

  public void runShooter (double speed) {
    //Set Motor Speeds
    shootMot1.set(speed);
    shootMot2.set(speed);
  }

  public double shootEquation () {
    double X = Vision.triangulate();
    return 0.000018717948718
          - 0.000648310023310 * X
          + 0.008542191142190 * Math.pow(X, 2)
          - 0.051929778554770 * Math.pow(X, 3)
          + 0.146411118881092 * Math.pow(X, 4)
          + 0.645400000000031 * Math.pow(X, 5);
  }

  public static boolean liftInit() {
    if (botLimitSwitch.get()){
      liftMot.set(0.02);;
    }
    if (!botLimitSwitch.get()){
      liftMot.set(0.02);
      liftEnc.setPosition(0);
      return true;
    }
    return false;
  }

  public static void runLift(double speed) {
    speed = -speed;
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
  }

  public void stopShooter () {
    shootMot1.set(0.0);
    shootMot2.set(0.0);
  }

  public DriverStation.Alliance readBall () {
    Color detectedColor = m_colorSensor.getColor();



    double IR = m_colorSensor.getIR();
    double red = detectedColor.red;
    double blue = detectedColor.blue;

    if (IR > 15) {
      SmartDashboard.putBoolean("has_ball", true);
    }
    else SmartDashboard.putBoolean("has_ball", false);

    if (red >= 0.4 && red <= 0.6 && blue <= 0.2) {
      return DriverStation.Alliance.Red;
    }

    else if (blue >= 0.33 && blue <= 0.5 && red <= 0.23) {
      return DriverStation.Alliance.Blue;
    }

    else return null;


  }

  public boolean isBall () {
    if (DriverStation.getAlliance() == readBall()){
      return true;
    }
    else {return false;}
  }

  public static double distanceToAngle () {
    return 0.0;
  }

  //PID controller for lift
  public static double liftPIDout() {
    double X = Vision.triangulate();
    double tarAngle = -0.001923076923076 - 
                      0.039335664335646 * X 
                      - 0.333391608391340 * Math.pow(X, 2)
                      + 3.076398601396759  * Math.pow(X, 3)
                      - 9.942540792534867 * Math.pow(X, 4)
                      + 38.545454545447484 * Math.pow(X, 5);
    MathUtil.clamp(tarAngle, 3.0 , 110.0);
    SmartDashboard.putNumber("TarAngle", tarAngle);
    if (Vision.get_lltarget()) {
      return MathUtil.clamp(testPID.calculate(MathUtil.clamp(-liftEnc.getPosition(), 0.0, 110.0),tarAngle), -0.1, 0.1);
    }
    else {return 0.0;}
  }

  
 

}
