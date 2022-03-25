// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.RobotContainer;
import frc.robot.Constants.driverConstants;

public class Drive extends CommandBase {
  private final Chassis m_Chassis;
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
    double forward = RobotContainer.m_DriverController.getRawAxis(driverConstants.X_ROTATE);
    double turn = RobotContainer.m_DriverController.getRawAxis(driverConstants.Y_AXIS);

    //Deadban
    turn = (Math.abs(turn) <= 0.1) ? 0 : turn;
    forward = (Math.abs(forward) <= 0.1) ? 0 : forward; 

    //Final Speed
    double leftMSpeed = (forward + turn);
    double rightMSpeed = (forward - turn);

    //Command call
    SmartDashboard.putNumber("Driver L speed", leftMSpeed);
    SmartDashboard.putNumber("Driver R speed", rightMSpeed);
    m_Chassis.drive(leftMSpeed, rightMSpeed);
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
