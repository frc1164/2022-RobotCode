// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//REV imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//Robot imports
import frc.robot.Constants;
import frc.robot.Constants.motorConstants;

public class Shooter extends SubsystemBase {
  public CANSparkMax shooterMot1,shooterMot2, indexMot;
  public Shooter() {
    shooterMot1 = new CANSparkMax(motorConstants.SPEED_CONT16, MotorType.kBrushless);
    shooterMot2 = new CANSparkMax(motorConstants.SPEED_CONT17, MotorType.kBrushless);

    indexMot = new CANSparkMax(motorConstants.SPEED_CONT18, MotorType.kBrushless);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIndex (double speed) {
    indexMot.set(speed);
  }

  public void runShooter (double speed) {
    shooterMot1.set(speed);
    shooterMot2.set(motorConstants.IS_REVERSED * speed);
  }
}
