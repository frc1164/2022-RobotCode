// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//WPI imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.driverConstants;
//Constants imports
import frc.robot.Constants.xBoxConstants;

//Commands imports
import frc.robot.commands.Drive;
import frc.robot.commands.Shoot;

//Subsystems imports
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Controller declares
  public static XboxController m_OperatorController;
  public static Joystick m_DriverController;

  //Commands declares
  private final Drive m_Drive;
  private final Shoot m_Shoot;

  //Subsystem declares
  private final Chassis m_Chassis;
  private final Shooter m_Shooter;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Subsystems init
    m_Chassis = new Chassis();
    m_Shooter = new Shooter();

    //Controllers init
    m_OperatorController = new XboxController(xBoxConstants.OPERATOR_PORT);
    m_DriverController = new Joystick(driverConstants.DRIVER_PORT);

    //Commands init
    m_Drive = new Drive(m_Chassis);
    m_Shoot = new Shoot(m_Shooter);

    //Set Default Commands
    m_Chassis.setDefaultCommand(m_Drive);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_OperatorController, xBoxConstants.A_BUTTON).whileHeld(m_Shoot);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
