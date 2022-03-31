// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.motorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber extends SubsystemBase {
  private static CANSparkMax angleMot;
  private CANSparkMax winchMot;
  private static DigitalInput topLim, botLim, backLim, frontLim;
  public Climber() {
    winchMot = new CANSparkMax(motorConstants.SPEED_CONT22, MotorType.kBrushless);
    angleMot = new CANSparkMax(motorConstants.SPEED_CONT23, MotorType.kBrushless);

    winchMot.setIdleMode(IdleMode.kBrake);
    angleMot.setIdleMode(IdleMode.kBrake);

    winchMot.setInverted(motorConstants.CLIMB_INVERTED);

    topLim = new DigitalInput(2);
    botLim = new DigitalInput(3);

    backLim = new DigitalInput(6);
    frontLim = new DigitalInput(7);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Top Limit", topLim.get());
    SmartDashboard.putBoolean("Bottom Limit", botLim.get());
    SmartDashboard.putBoolean("back Limit", backLim.get());
    SmartDashboard.putBoolean("front Limit", frontLim.get());
  }

  public void angleClimb (double speed) {
    if (speed > 0.0){
      if(frontLim.get()){
        angleMot.set(speed);
      }
      else {
        angleMot.set(0.0);
      }
    }
    else if (speed < 0.0) {
      if (backLim.get()){
         angleMot.set(speed);
      }
      else {
        angleMot.set(0.0);
      }
    }
    else {
      angleMot.set(0.0);
    }
  }

  public void runWinch (double speed) {
    if (speed > 0.1){
      if(topLim.get()){
        winchMot.set(speed);
      }
      else {
        winchMot.set(0.0);
      }
    }
    else if (speed < -0.1) {
      if (botLim.get()){
         winchMot.set(speed);
      }
      else {
        winchMot.set(0.0);
      }
    }
    else {
      winchMot.set(0.0);
    }
    SmartDashboard.putNumber("Speed", speed);
  }
}
