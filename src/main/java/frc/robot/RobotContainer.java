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

//Robot imports
import frc.robot.subsystems.Chassis;
import frc.robot.Constants.driverConstants;

//Constants imports
import frc.robot.Constants.xBoxConstants;

//Commands imports
import frc.robot.commands.Drive;
import frc.robot.commands.RunLift;
import frc.robot.commands.Shoot;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunConveyor;
import frc.robot.commands.InitCommands.LiftInit;

//Subsystems imports
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

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
  private final RunLift m_RunLift;
  private final LiftInit m_LiftInit;
  private final RunFeeder  m_RunFeeder;
  private final RunConveyor m_RunConveyor;
  private final RunIntake m_RunIntake;

  //Subsystem declares
  private final Chassis m_Chassis;
  public static Shooter m_Shooter;
  private final Intake m_Intake;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Subsystems init
    m_Chassis = new Chassis();
    m_Shooter = new Shooter();
    m_Intake = new Intake();

    //Controllers init
    m_OperatorController = new XboxController(xBoxConstants.OPERATOR_PORT);
    m_DriverController = new Joystick(driverConstants.DRIVER_PORT);

    //Commands init
    m_Drive = new Drive(m_Chassis);
    m_Shoot = new Shoot(m_Shooter);
    m_RunLift = new RunLift(m_Shooter);
    m_LiftInit = new LiftInit(m_Shooter);
    m_RunFeeder = new RunFeeder();
    m_RunConveyor = new RunConveyor();
    m_RunIntake = new RunIntake(m_Intake);

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
    new JoystickButton(m_OperatorController, xBoxConstants.B_BUTTON).whileHeld(m_RunConveyor);
    new JoystickButton(m_OperatorController, xBoxConstants.X_BUTTON).whenPressed(m_LiftInit);
    new JoystickButton(m_OperatorController, xBoxConstants.Y_BUTTON).whileHeld(m_RunIntake);
    new JoystickButton(m_OperatorController, 5).whileHeld(m_RunLift);
    new JoystickButton(m_OperatorController, 6).whileHeld(m_RunFeeder);

    




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
