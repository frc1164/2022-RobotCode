// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//WPI imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//REV imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

//Robot imports
import frc.robot.Constants.motorConstants;

public class Shooter extends SubsystemBase {
  public CANSparkMax shootMot1,shootMot2, indexMot;
  public RelativeEncoder shootEnc1, shootEnc2;
  public Shooter() {
    //Shooter Motors
    shootMot1 = new CANSparkMax(motorConstants.SPEED_CONT16, MotorType.kBrushless);
    shootMot2 = new CANSparkMax(motorConstants.SPEED_CONT17, MotorType.kBrushless);

    shootMot1.setIdleMode(IdleMode.kCoast);
    shootMot2.setIdleMode(IdleMode.kCoast);

    shootMot1.setInverted(motorConstants.SHOOTMOT1_INVERT);

    //Shooter Encoders
    shootEnc1 = shootMot1.getEncoder();
    shootEnc2 = shootMot2.getEncoder();

    //Smartdashboard
    SmartDashboard.putNumber("Shooter Motor Velocity Input", shootEnc1.getVelocity());

    //Indexer Motor
    indexMot = new CANSparkMax(motorConstants.SPEED_CONT18, MotorType.kBrushless);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIndex (double speed) {
    indexMot.set(speed);
  }

  public void runShooter () {
    //Get values from Smartdashboard
    double speed = SmartDashboard.getNumber("Shooter Motor Velocity Input", 0.0);

    //Set Motor Speeds
    shootMot1.set(speed);
    shootMot2.set(speed);

    SmartDashboard.putNumber("Motor 1 Velocity", shootEnc1.getVelocity());
    SmartDashboard.putNumber("Motor 2 Velocity", shootEnc2.getVelocity());
  }

  public void stopShooter () {
    shootMot1.set(0.0);
    shootMot2.set(0.0);
  }
}
