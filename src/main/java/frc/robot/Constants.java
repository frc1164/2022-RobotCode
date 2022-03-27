// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
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
        public static final int SPEED_CONT19 = 1;


        public static final boolean LFMOT_INVERT = true;
        public static final boolean LBMOT_INVERT = true;
        public static final boolean RFMOT_INVERT = false;
        public static final boolean RBMOT_INVERT = false;

        public static final boolean SHOOTMOT1_INVERT = true;
        public static final boolean SHOOTMOT2_INVERT = false;

        public static final double INDEX_MOTOR_SPEED = .45;
    }


    public static final class limitSwitchConstants {
        public static final int TOP_LIMIT_SWITCH_PORT = 0;
        public static final int BOTTOM_LIMIT_SWITCH_PORT = 1;

        
    }

    public static final class xBoxConstants {
        public static final int OPERATOR_PORT = 1;
        public static final int LY_AXIS = 1;
        public static final int LX_AXIS = 0;
        public static final int A_BUTTON = 0;
        public static final int B_BUTTON = 1;
        public static final int X_BUTTON = 2;

    }

    public static final class driverConstants{
        public static final int DRIVER_PORT = 0;
        public static final int X_AXIS = 0;
        public static final int Y_AXIS = 1;
        public static final int Z_AXIS = 2;
        public static final int X_ROTATE = 3;
    }
}
