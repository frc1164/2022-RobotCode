// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.LinkedList;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.controller.PIDController;

public class Vision extends SubsystemBase {
  public static NetworkTable table, distTable;
  public static NetworkTableEntry tx, ty, ta, tv, td;
  LinkedList<Double> LLvalues = new LinkedList<Double>();

  //Declare PID contoller and values
  private ShuffleboardTab tab = Shuffleboard.getTab("PID LL Settings");
  public static double P, I, D, dP, min_Command;
  public static double PIDout, steeringAdjust;
  static PIDController testPID = new PIDController(P, I, D);
  Number cameraMode = 1;
  
  public Vision() {
    //Sets up Lime Light Network Tables
    table = NetworkTableInstance.getDefault().getTable("limelight");
    distTable = NetworkTableInstance.getDefault().getTable("Smartdashboard");

    //limelight data
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");

    //calculated data
    td = SmartDashboard.getEntry("Distance from target");

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);
    

    //PID Init
    PIDout = 0.0;
    P = 0.8;
    I = 0.016;
    D = 0.8;
    testPID.setPID(P, I, D);
    testPID.setSetpoint(0.0);
    testPID.enableContinuousInput(-29.8, 29.8);

    //Distance PID Init
    dP = 0.01;
    min_Command = 0.025;
    steeringAdjust = 0.0;
  }

  //Method for LimeLight valid target
  public static boolean get_lltarget() {
    double LLt = tv.getDouble(0.0);
    
    if (LLt == 1) {
      return true;
    }

    else {
      return false;
    }
  }

  //Method for LimeLight x_axis
  public static double get_llx() {
    double LLx = tx.getDouble(0.0);
    return LLx;
  }

  //Method for LimeLight y_axis
  public static double get_lly() {
    double LLy = ty.getDouble(0.0);
    return LLy;
  }

  //Method for LimeLight area
  public static double get_llarea() {
    double LLarea = ta.getDouble(0.0);
    return LLarea;
  }

  //Triangulates the distance from goal plane
  public static double triangulate(){
    if (get_lltarget() == true){
    double distance = 76 / Math.tan(Math.toRadians(29.6641 + get_lly())); 
    return distance;
    }
    else{
      return 0;
    }
  }

  //Displays LimeLight Values
  public void printLLvalues() {
    SmartDashboard.putBoolean("Object Detected", get_lltarget());
    SmartDashboard.putNumber("LimelightX", get_llx());
    SmartDashboard.putNumber("LimelightY", get_lly());
    SmartDashboard.putNumber("LimelightArea", get_llarea());
    SmartDashboard.putNumber("Distance from target", triangulate());
  }

  //PID controller for Centering
  public static double centerPIDout() {
    if (get_lltarget()) {
       return testPID.calculate(get_llx());
      }
      else {return 0.1;}
  }

  //Distance PID controller
  public static double distancePID() {
    double botDistance = td.getDouble(0.0);
    SmartDashboard.putNumber("Bot Distance", botDistance);
    double distanceError = 54.0 - botDistance; 
    SmartDashboard.putNumber("Distance Error", distanceError);
    if (distanceError > 5.0) {
      steeringAdjust = dP*distanceError - min_Command;
    }
    if (distanceError < -5.0) {
      steeringAdjust = dP*distanceError + min_Command;
    }
    if (distanceError >= -5.0 && distanceError <= 5.0){
      steeringAdjust = 0.0;
    }
    SmartDashboard.putNumber("Steering Adjust", steeringAdjust);
    return steeringAdjust;
  }


  @Override
  public void periodic() {
    printLLvalues();
  }
}
