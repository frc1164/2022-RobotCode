// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.motorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Intake extends SubsystemBase {
  public CANSparkMax beatLift, beatRoll;
  public static VictorSPX conveyorMot;
  public DigitalInput topLim, botLim;
  public boolean stop;
  public Intake() {
    beatLift = new CANSparkMax(motorConstants.BEATER_LIFT, MotorType.kBrushless);
    beatRoll = new CANSparkMax(motorConstants.BEATER_ROLL, MotorType.kBrushless);
    conveyorMot = new VictorSPX(motorConstants.CONVEYOR);

    beatLift.setIdleMode(IdleMode.kBrake);
    beatRoll.setIdleMode(IdleMode.kCoast);

    topLim = new DigitalInput(4);
    botLim = new DigitalInput(5);

    stop = false;


    SmartDashboard.putNumber("Conveyor Motor Input", 0.3);
    SmartDashboard.putNumber("Intake Motor Input", 0.0);
  }

  public static void runConveyor () {
    double speed = SmartDashboard.getNumber("Conveyor Motor Input", 0.0);
    conveyorMot.set(ControlMode.PercentOutput, speed);
  }

  public void runIntake () {
    double speed = SmartDashboard.getNumber("Intake Motor Input", 0.0);
    beatRoll.set(0.4);
  }

  public boolean beaterLift () {
    System.out.println("Method start");
    if (botLim.get()){
      beatLift.set(0.15);
      System.out.println("set motor speed");
    }
    if (!botLim.get()){
      beatLift.set(0.0);
      System.out.print("Bot Lim pressed");
      return true;
    }
    return false;
    }
  
public boolean beaterUp () {
  System.out.println("Method start");
  if (topLim.get()){
    beatLift.set(-0.15);
    System.out.println("set motor speed");
  }
  if (!topLim.get()){
    beatLift.set(0.0);
    System.out.print("Bot Lim pressed");
    return true;
  }
  return false;
}

public void endLift (){
  beatLift.set(0);
}

public void manualLift (double speed) {
  if (speed > 0 && topLim.get()){
    beatLift.set(speed);
  }
  if (speed < 0 && botLim.get()){
    beatLift.set(speed);
  }
  else beatLift.set(0);
}

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Beater top lim", topLim.get());
    SmartDashboard.putBoolean("Beater bot lim", botLim.get());

  }
}
