package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;

import java.rmi.server.RMIFailureHandler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.characterizationConstants;
import frc.robot.Constants.motorConstants;

import com.ctre.phoenix.led.RgbFadeAnimation;
import com.kauailabs.navx.frc.AHRS;

public class Chassis extends SubsystemBase {

  private CANSparkMax lfMot = new CANSparkMax(motorConstants.SPEED_CONT12, MotorType.kBrushless);
  private CANSparkMax lbMot = new CANSparkMax(motorConstants.SPEED_CONT13, MotorType.kBrushless);
  private CANSparkMax rfMot = new CANSparkMax(motorConstants.SPEED_CONT14, MotorType.kBrushless);
  private CANSparkMax rbMot = new CANSparkMax(motorConstants.SPEED_CONT15, MotorType.kBrushless);

  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(lfMot, lbMot);

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(rfMot, rbMot);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  private final RelativeEncoder m_leftEncoder = lfMot.getEncoder();
  // The right-side drive encoder
  private final RelativeEncoder m_rightEncoder = rfMot.getEncoder();

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB);

  // Odometry class for tracking robot pose
  private final DifferentialDrivePoseEstimator odometer = new DifferentialDrivePoseEstimator(characterizationConstants.kDriveKinematics, m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), null);
  
  private final Field2d m_field;
  
  /** Creates a new DriveSubsystem. */
  public Chassis() {

    m_field = new Field2d();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    lfMot.restoreFactoryDefaults(true);
    lbMot.restoreFactoryDefaults(true);
    rfMot.restoreFactoryDefaults(true);
    rbMot.restoreFactoryDefaults(true);
    lfMot.setIdleMode(IdleMode.kCoast);
    lbMot.setIdleMode(IdleMode.kCoast);
    rfMot.setIdleMode(IdleMode.kCoast);
    rbMot.setIdleMode(IdleMode.kCoast);


    m_leftMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setPositionConversionFactor(characterizationConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setPositionConversionFactor(characterizationConstants.kEncoderDistancePerPulse);

    resetEncoders();
  
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometer.getEstimatedPosition();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometer.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

@Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometer.update(
        m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    
    SmartDashboard.putString("Alliance Color", DriverStation.getAlliance().name());
    m_field.setRobotPose(this.getPose());
    SmartDashboard.putData(m_field);
  }
}