// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.characterizationConstants;
/** Add your docs here. */

public class RobotOdometry {
    private static final RobotOdometry robotOdometry = new RobotOdometry();
    private DifferentialDrivePoseEstimator estimator;
  
    private RobotOdometry() {
      estimator =
          new DifferentialDrivePoseEstimator(
              characterizationConstants.kDriveKinematics,
              new Rotation2d(),
              0,
              0,
              new Pose2d());
    }
  
    public static RobotOdometry getInstance() {
      return robotOdometry;
    }
  
    public DifferentialDrivePoseEstimator getPoseEstimator() {
      return estimator;
    }
  }