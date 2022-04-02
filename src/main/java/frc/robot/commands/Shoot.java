// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//WPI imports
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.motorConstants;
import frc.robot.subsystems.Intake;
//Robot imports
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
  private final Shooter m_Shooter;
  private final Intake m_Intake;
  public Shoot(Shooter m_Shooter, Intake m_Intake) {
    this.m_Shooter = m_Shooter;
    this.m_Intake = m_Intake;
    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Shooter.isBall()){
      m_Shooter.runShooter(0.85);
    }
    else {
      m_Shooter.runShooter(0.85);
    }
    m_Shooter.runFeeder(motorConstants.FEEDER_MOTOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.runFeeder(0.0);
    m_Shooter.runShooter(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
