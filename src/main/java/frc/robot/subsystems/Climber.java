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
  private static DigitalInput topLim, botLim, angleLim;
  public Climber() {
    winchMot = new CANSparkMax(motorConstants.SPEED_CONT22, MotorType.kBrushless);
    angleMot = new CANSparkMax(motorConstants.SPEED_CONT23, MotorType.kBrushless);

    winchMot.setIdleMode(IdleMode.kBrake);
    angleMot.setIdleMode(IdleMode.kBrake);

    winchMot.setInverted(motorConstants.CLIMB_INVERTED);

    topLim = new DigitalInput(2);
    botLim = new DigitalInput(3);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Top Limit", topLim.get());
    SmartDashboard.putBoolean("Bottom Limit", botLim.get());
  }

  public void runWinch (double speed) {
    if(topLim.get() && speed > 0){
      winchMot.set(speed);
    } else if (botLim.get() && speed < 0){
      winchMot.set(speed);
    }
    else {
      winchMot.set(0.0);
    }
  }

  public static void angleClimb (double speed) {
    if(topLim.get() && botLim.get()){
      angleMot.set(speed);
    }
  }
}
