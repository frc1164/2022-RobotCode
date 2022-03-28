// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.motorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
  private CANSparkMax beatLift, beatRoll, conveyorMot;
  public Intake() {
    beatLift = new CANSparkMax(motorConstants.BEATER_LIFT, MotorType.kBrushless);
    beatRoll = new CANSparkMax(motorConstants.BEATER_ROLL, MotorType.kBrushless);
    conveyorMot = new CANSparkMax(motorConstants.CONVEYOR, MotorType.kBrushless);

    SmartDashboard.putNumber("Conveyor Motor Input", 0.0);
    SmartDashboard.putNumber("Intake Motor Input", 0.0);
  }

  public void runConveyor () {
    double speed = SmartDashboard.getNumber("Conveyor Motor Input", 0.0);
    conveyorMot.set(speed);
  }

  public void runIntake () {
    double speed = SmartDashboard.getNumber("Intake Motor Input", 0.0);
    beatRoll.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
