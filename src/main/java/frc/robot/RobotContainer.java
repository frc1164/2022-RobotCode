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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

//Robot imports
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
import frc.robot.commands.InitCommands.ManualIntake;
import frc.robot.commands.runClimb;
import frc.robot.commands.AngleClimb;
import frc.robot.commands.AutoLift;
import frc.robot.commands.AutoShoot;
import frc.robot.subsystems.Climber;
import frc.robot.commands.RunBeatUp;
import frc.robot.commands.EndLift;
import frc.robot.commands.InitCommands.ManualIntake;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoDistance;
import frc.robot.commands.AutoGoal;


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
  private final RunFeeder m_RunFeeder;
  private final RunConveyor m_RunConveyor;
  private final RunIntake m_RunIntake;
  private final RunBeaterLift m_RunBeaterLift;
  private final AngleClimb m_angleClimb;
  private final CenterDistance m_CenterDistance;
  private final CenterGoal m_CenterGoal;
  private final AutoLift m_AutoLift;
  private final RunBeatUp m_RunBeatUp;
  private final ManualIntake m_ManualIntake;
  private final AutoShoot m_AutoShoot;
  private final AutoDistance m_AutoDistance;
  private final AutoGoal m_AutoGoal;


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
    m_Shoot = new Shoot(m_Shooter, m_Intake);
    m_ReadBall = new ReadBall(m_Shooter);
    m_RunLift = new RunLift(m_Shooter);
    m_RunFeeder = new RunFeeder(m_Shooter);
    m_RunConveyor = new RunConveyor();
    m_RunIntake = new RunIntake(m_Intake);
    m_RunBeaterLift = new RunBeaterLift(m_Intake);
    m_angleClimb = new AngleClimb(m_Climber);
    m_CenterDistance = new CenterDistance();
    m_CenterGoal = new CenterGoal();
    m_AutoLift = new AutoLift(m_Shooter);
    m_RunClimb = new runClimb(m_Climber);
    m_RunBeatUp = new RunBeatUp(m_Intake);
    m_ManualIntake = new ManualIntake(m_Intake);
    m_AutoShoot = new AutoShoot();
    m_AutoGoal = new AutoGoal();
    m_AutoDistance = new AutoDistance();

    //Set Default Commands
    m_Chassis.setDefaultCommand(m_Drive);
    m_Shooter.setDefaultCommand(m_ReadBall);
    m_Intake.setDefaultCommand(m_ManualIntake);


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
    new JoystickButton(m_OperatorController, xBoxConstants.A_BUTTON).whileHeld(new ParallelCommandGroup(m_CenterGoal, m_CenterDistance));
    new JoystickButton(m_OperatorController, xBoxConstants.Y_BUTTON).whileHeld(m_RunIntake);
    new JoystickButton(m_OperatorController, 8).whenPressed(m_RunBeaterLift);
    new JoystickButton(m_OperatorController, 7).whenPressed(m_RunBeatUp);
    //new JoystickButton(m_OperatorController, 5).whenPressed(m_angleClimb);
    new JoystickButton(m_OperatorController, 6).whenPressed(m_RunClimb);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   // Create a voltage constraint to ensure we don't accelerate too fast
    return new SequentialCommandGroup(m_AutoGoal, m_AutoDistance, m_Shoot);
  }
}
