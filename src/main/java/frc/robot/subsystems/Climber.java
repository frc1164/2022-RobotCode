// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.motorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber extends SubsystemBase {
  public CANSparkMax winchMot, angleMot;
  public DigitalInput topLim, botLim, angleLim;
  public Climber() {
    winchMot = new CANSparkMax(motorConstants.SPEED_CONT22, MotorType.kBrushless);
    angleMot = new CANSparkMax(motorConstants.SPEED_CONT23, MotorType.kBrushless);

    winchMot.setIdleMode(IdleMode.kBrake);
    angleMot.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runWinch (double speed) {
    winchMot.set(speed);
  }
}
