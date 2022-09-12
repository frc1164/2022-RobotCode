// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

import edu.wpi.first.wpilibj.I2C;

public final class Constants {
    public static final class motorConstants {
        public static final int SPEED_CONT3 = 3;
        public static final int SPEED_CONT4 = 4;
        public static final int SPEED_CONT5 = 5;
        public static final int SPEED_CONT6 = 6;
        public static final int SPEED_CONT7 = 7;
        public static final int SPEED_CONT8 = 8;
        public static final int SPEED_CONT9 = 9;
        public static final int SPEED_CONT10 = 10;
        public static final int SPEED_CONT11 = 11;
        public static final int SPEED_CONT12 = 12;
        public static final int SPEED_CONT13 = 13;
        public static final int SPEED_CONT14 = 14;
        public static final int SPEED_CONT15 = 15;
        public static final int SPEED_CONT16 = 16;
        public static final int SPEED_CONT17 = 17;
        public static final int SPEED_CONT18 = 18;
        public static final int SHOOTER_LIFT = 19;
        public static final int BEATER_ROLL = 20;
        public static final int CONVEYOR = 21;
        public static final int SPEED_CONT22 = 22;
        public static final int SPEED_CONT23 = 23;
        public static final int BEATER_LIFT = 24;

        public static final boolean LFMOT_INVERT = true;
        public static final boolean LBMOT_INVERT = true;
        public static final boolean RFMOT_INVERT = false;
        public static final boolean RBMOT_INVERT = false;
        public static final boolean SHOOTMOT1_INVERT = true;
        public static final boolean SHOOTMOT2_INVERT = false;
        public static final boolean CLIMB_INVERTED = false;
        public static final boolean SHOOTER_LIFT_INVERT = false;

        public static final double FEEDER_MOTOR_SPEED = 1;
    }


    public static final class limitSwitchConstants {
        public static final int TOP_LIMIT_SWITCH_PORT = 0;
        public static final int BOTTOM_LIMIT_SWITCH_PORT = 1;

        public static final int SPEED_CONT22 = 22;
        public static final int SPEED_CONT23 = 23;

        public static final boolean CLIMB_INVERTED = true;
    }

    public static final class xBoxConstants {
        public static final int OPERATOR_PORT = 1;
        public static final int LY_AXIS = 1;
        public static final int LX_AXIS = 0;
        public static final int A_BUTTON = 1;
        public static final int B_BUTTON = 2;
        public static final int X_BUTTON = 3;
        public static final int Y_BUTTON = 4;

    }

    public static final class driverConstants{
        public static final int DRIVER_PORT = 0;
        public static final int X_AXIS = 4;
        public static final int Y_AXIS = 1;
        public static final int Z_AXIS = 2;
        public static final int X_ROTATE = 3;
    }

    public static final class colorSensorConstants{
        public static final I2C.Port i2cPort = I2C.Port.kOnboard;
    }
    
    public static final class characterizationConstants {
        public static final double ksVolts = 0.17931;
        public static final double kvVoltSecondsPerMeter = 1.3291;
        public static final double kaVoltSecondsSquaredPerMeter = 0.15985;
        public static final double kPDriveVel = 1.6738;
        public static final double kTrackwidthMeters = 0.5715;
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final double kEncoderDistancePerPulse = 0.11969;


        public static final DifferentialDriveKinematics kDriveKinematics = 
        new DifferentialDriveKinematics(kTrackwidthMeters);
    }
}
