// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//Robot imports
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.motorConstants;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunFeeder extends CommandBase {
  Shooter m_Shooter;
  public RunFeeder(Shooter m_Shooter) {
    this.m_Shooter = m_Shooter;
    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Shooter.runFeeder(motorConstants.FEEDER_MOTOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.runFeeder(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
