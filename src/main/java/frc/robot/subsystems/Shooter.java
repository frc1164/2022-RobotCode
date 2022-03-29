// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//WPI imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

//REV imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

//Robot imports
import frc.robot.Constants.motorConstants;
import frc.robot.Constants.limitSwitchConstants;

public class Shooter extends SubsystemBase {
  public CANSparkMax shootMot1,shootMot2; 
  public static CANSparkMax feederMot, liftMot;
  public RelativeEncoder shootEnc1, shootEnc2;
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

    if (topLimitSwitch.get() && botLimitSwitch.get()){
      liftMot.set(speed);

    } else {liftMot.set(0.0);}

    SmartDashboard.putNumber("Lift position", liftEnc.getPosition());
  }

  public void stopShooter () {
    shootMot1.set(0.0);
    shootMot2.set(0.0);
  }
}
