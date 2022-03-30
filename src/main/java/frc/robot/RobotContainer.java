// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.dense.block.MatrixOps_DDRB;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.xBoxConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.DashSpeed;
import frc.robot.commands.runClimb;
import frc.robot.commands.angleClimb;
import frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  public static XboxController m_OperatorController;

  private final Drive m_Drive;
  private final DashSpeed m_DashSpeed;
  private final runClimb m_RunClimb;
  private final Chassis m_Chassis;
  private final Climber m_Climber;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings

    m_Chassis = new Chassis();
    m_Climber = new Climber();

    //Define Controllers
    m_OperatorController = new XboxController(xBoxConstants.OPERATOR_PORT);

    //Define Commands
    m_Drive = new Drive(m_Chassis);
    m_DashSpeed = new DashSpeed(m_Chassis);
    m_RunClimb = new runClimb(m_Climber);

    //Set Default Commands
    //m_Chassis.setDefaultCommand(m_Drive);
    m_Climber.setDefaultCommand(m_RunClimb);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_OperatorController, xBoxConstants.A_BUTTON).whileHeld(m_DashSpeed);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
