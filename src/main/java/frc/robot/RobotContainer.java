// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//WPI imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;

//Robot imports
import frc.robot.Constants.driverConstants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

//Constants imports
import frc.robot.Constants.driverConstants;
import frc.robot.Constants.xBoxConstants;
import frc.robot.commands.AngleClimb;
import frc.robot.commands.CenterDistance;
import frc.robot.commands.CenterGoal;
//Commands imports
import frc.robot.commands.Drive;
import frc.robot.commands.ReadBall;
import frc.robot.commands.RunBeaterLift;
import frc.robot.commands.RunLift;
import frc.robot.commands.Shoot;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunConveyor;
import frc.robot.commands.InitCommands.LiftInit;
import frc.robot.commands.runClimb;
import frc.robot.commands.angleClimb;
import frc.robot.subsystems.Climber;

//Subsystems imports
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

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
  private final ReadBall m_ReadBall;
  private final RunLift m_RunLift;
  private final RunFeeder  m_RunFeeder;
  private final RunConveyor m_RunConveyor;
  private final RunIntake m_RunIntake;
  private final RunBeaterLift m_RunBeaterLift;
  private final AngleClimb m_AngleClimb;
  private final CenterDistance m_CenterDistance;
  private final CenterGoal m_CenterGoal;

  //Subsystem declares
  public final Shooter m_Shooter;
  private final Intake m_Intake;
  private final Vision m_Vision;
  private final runClimb m_RunClimb;
  private final Chassis m_Chassis;
  private final Climber m_Climber;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Subsystems init
    m_Chassis = new Chassis();
    m_Shooter = new Shooter();
    m_Vision = new Vision();
    m_Intake = new Intake();
    m_Climber = new Climber();

    //Controllers init
    m_OperatorController = new XboxController(xBoxConstants.OPERATOR_PORT);
    m_DriverController = new Joystick(driverConstants.DRIVER_PORT);

    //Commands init
    m_Drive = new Drive(m_Chassis);
    m_Shoot = new Shoot(m_Shooter);
    m_ReadBall = new ReadBall(m_Shooter);
    m_RunLift = new RunLift(m_Shooter);
    m_RunFeeder = new RunFeeder();
    m_RunConveyor = new RunConveyor();
    m_RunIntake = new RunIntake(m_Intake);
    m_RunBeaterLift = new RunBeaterLift(m_Intake);
    m_AngleClimb = new AngleClimb(m_Climber);
    m_CenterDistance = new CenterDistance();
    m_CenterGoal = new CenterGoal();

    //Set Default Commands
    m_Chassis.setDefaultCommand(m_Drive);
    m_Shooter.setDefaultCommand(m_ReadBall);
    m_Intake.setDefaultCommand(m_RunBeaterLift);
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
    new JoystickButton(m_OperatorController, xBoxConstants.B_BUTTON).whileHeld(m_Shoot);
    new JoystickButton(m_OperatorController, xBoxConstants.Y_BUTTON).whileHeld(new ParallelCommandGroup(new Shoot(m_Shooter), 
                                                                               new CenterGoal()));
    new JoystickButton(m_OperatorController, xBoxConstants.A_BUTTON).whileHeld(m_CenterGoal);
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
