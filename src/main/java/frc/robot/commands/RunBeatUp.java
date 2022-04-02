// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RunBeatUp extends CommandBase {
  Intake m_Intake;
  boolean isStop;
  public RunBeatUp(Intake m_Intake) {
    this.m_Intake = m_Intake;
    addRequirements(m_Intake);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isStop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isStop = m_Intake.beaterUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.beatLift.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isStop;
  }
}
