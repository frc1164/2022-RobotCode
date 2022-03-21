// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//WPILIB imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//REVRobotics imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

//local imports
import frc.robot.Constants.motorConstants;

public class Chassis extends SubsystemBase {
  private final CANSparkMax LFMotor, LBMotor, RFMotor, RBMotor;


  public Chassis() {
    //Initialize motors
    LFMotor = new CANSparkMax(motorConstants.SPEED_CONT12, MotorType.kBrushless);
    LBMotor = new CANSparkMax(motorConstants.SPEED_CONT13, MotorType.kBrushless);
    RFMotor = new CANSparkMax(motorConstants.SPEED_CONT14, MotorType.kBrushless);
    RBMotor = new CANSparkMax(motorConstants.SPEED_CONT15, MotorType.kBrushless);

    //Set brake
    LFMotor.setIdleMode(IdleMode.kBrake);
    LBMotor.setIdleMode(IdleMode.kBrake);
    RFMotor.setIdleMode(IdleMode.kBrake);
    RBMotor.setIdleMode(IdleMode.kBrake);
      
    }

  public void drive(double lSpeed, double rSpeed) {
    LFMotor.set(motorConstants.IS_REVERSED * lSpeed);
    LBMotor.set(motorConstants.IS_REVERSED * lSpeed);

    RFMotor.set(rSpeed);
    RBMotor.set(rSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
