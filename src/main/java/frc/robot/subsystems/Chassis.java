// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//PheonixTuner imports
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;

//WPILIB imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//REVRobotics imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

//local imports
import frc.robot.Constants.motorConstants;

public class Chassis extends SubsystemBase {
  private final CANSparkMax testController, testController_2, LFMotor, LBMotor, RFMotor, RBMotor;
  private final RelativeEncoder testEncoder, testEncoder_2;


  public Chassis() {
    //Initialize motors
    testController = new CANSparkMax(motorConstants.SPEED_CONT16, MotorType.kBrushless);
    testController_2 = new CANSparkMax(motorConstants.SPEED_CONT17, MotorType.kBrushless);
    LFMotor = new CANSparkMax(motorConstants.SPEED_CONT12, MotorType.kBrushless);
    LBMotor = new CANSparkMax(motorConstants.SPEED_CONT13, MotorType.kBrushless);
    RFMotor = new CANSparkMax(motorConstants.SPEED_CONT14, MotorType.kBrushless);
    RBMotor = new CANSparkMax(motorConstants.SPEED_CONT15, MotorType.kBrushless);

    //Initialize encoders
    testEncoder = testController.getEncoder();
    testEncoder_2 = testController_2.getEncoder();

    //Set brake
    testController.setIdleMode(IdleMode.kBrake);
    testController_2.setIdleMode(IdleMode.kBrake);
    LFMotor.setIdleMode(IdleMode.kBrake);
    LBMotor.setIdleMode(IdleMode.kBrake);
    RFMotor.setIdleMode(IdleMode.kBrake);
    RBMotor.setIdleMode(IdleMode.kBrake);

    //Initialize input sections in Smartdashboard
    //SmartDashboard.putNumber("PlaceHolder (DO NOT USE)", testEncoder.getVelocity());
    SmartDashboard.putNumber("Test Motor(s) Velocity", testEncoder_2.getVelocity());
      
    }

  public void testMotor (double speed) {
    testController.set(speed);
    testController.set(speed);
    SmartDashboard.putNumber("Encoder", testEncoder.getPosition());
    SmartDashboard.putNumber("Velocity", testEncoder.getVelocity());
    SmartDashboard.putNumber("Motor Speed", testController.get());
  }

  public void drive(double lSpeed, double rSpeed) {
    LFMotor.set(motorConstants.IS_REVERSED * lSpeed);
    LBMotor.set(motorConstants.IS_REVERSED * lSpeed);

    RFMotor.set(rSpeed);
    RBMotor.set(rSpeed);
  }
  public void setSpeeds() {
    //Get values from Smartdashboard
    double lSpeed = SmartDashboard.getNumber("Test Motor(s) Velocity", 0.0);
    double rSpeed = SmartDashboard.getNumber("Test Motor(s) Velocity", 0.0);

    //Set Motor Speeds
    testController.set(motorConstants.IS_REVERSED * lSpeed);
    testController_2.set(rSpeed);

    SmartDashboard.putNumber("Motor 1 Velocity", testEncoder.getVelocity());
    SmartDashboard.putNumber("Motor 2 Velocity", testEncoder_2.getVelocity());
  }

  public void setZero() {
    //Set Motor Speeds
    testController.set(0);
    testController_2.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
