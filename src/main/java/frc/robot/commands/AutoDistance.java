// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.characterizationConstants;
import frc.robot.subsystems.Chassis;


import frc.robot.subsystems.Vision;

public class AutoDistance extends CommandBase {
  private static double forward = 0.0;
  private static double turn = 0.0;
  private Chassis m_Chassis;
  Timer m_Timer;
  public AutoDistance(Chassis m_Chassis) {
    this.m_Chassis = m_Chassis;
    m_Timer = new Timer();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    forward = Vision.distancePID();
    turn = Vision.centerPIDout();
    m_Chassis.arcadeDrive(forward, turn);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Chassis.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_Timer.get() > 5){
      return true;
    }
    return false;
  }
}
