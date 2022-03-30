// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Climber;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class runClimb extends CommandBase {
  Climber m_Climber;
  public runClimb(Climber m_Climber) {
    this.m_Climber = m_Climber;
    addRequirements(m_Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Climber.runWinch(RobotContainer.m_OperatorController.getRawAxis(1)/4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
