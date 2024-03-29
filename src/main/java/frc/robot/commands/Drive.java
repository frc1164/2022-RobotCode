// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.RobotContainer;
import frc.robot.Constants.driverConstants;
import frc.robot.commands.CenterDistance;
import frc.robot.commands.CenterGoal;

public class Drive extends CommandBase {
  private final Chassis m_Chassis;
  double forward, turn, leftMSpeed, rightMSpeed;
  /** Creates a new Drive. */
  public Drive(Chassis m_Chassis) {
    this.m_Chassis = m_Chassis;
    addRequirements(m_Chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Get drive values from controller
    double forward = RobotContainer.m_DriverController.getRawAxis(driverConstants.Y_AXIS);
    double turn = -RobotContainer.m_DriverController.getRawAxis(driverConstants.X_AXIS);

    //Deadban
    turn = (Math.abs(turn) <= 0.1) ? 0 : turn;
    forward = (Math.abs(forward) <= 0.1) ? 0 : forward; 

    if (turn == 0 && forward == 0){
      turn = turn + CenterGoal.centerSpeed;
      forward = forward + CenterDistance.distanceSpeed;
    }

    m_Chassis.arcadeDrive(forward, turn);
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
