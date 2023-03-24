// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /*
     * Hi guys - David (Mr. Rudo)
     */
    public static int FLCAN=3;
    public static int FRCAN=1;
    public static int BLCAN=4;
    public static int BRCAN=2;
    public static int EXTENDCAN=5;
    public static int ARMCAN=6;
    public static int PENUMATIC_IN = 6;
    public static int PENUMATIC_OUT = 7;

    public static Joystick LEFTJOY = new Joystick(0);
    public static Joystick RIGHTJOY = new Joystick(1);
    //public static Joystick RIGHTJOY = new Joystick(0);

    public static double KP_DRIVE_R = 1;
    public static double KP_DRIVE_X = 1;

    public static double KP_ARM = 1;
    public static double KI_ARM = 0.0002;
    public static double KD_ARM = 0.85;
    public static double ARM_KIZ = 0;
    public static double ARM_KMAX = 0.7;
    public static double ARM_KMIN = -0.2;
    public static double ARM_MAXRPM = 5000;
    public static double ARM_KFF= 0.1;


    public static double KS_ARM = 1;
    public static double KP_EXTEND = 0.1;
    public static double KS_EXTEND = 0.2;
    public static double ARM_CONTROL_SCALAR = 5;
    public static double ARM_DEG_TOL = 5;
    public static double EXTEND_CONTROL_SCALAR = 5;
    public static double EXTEND_DEG_TOL = 5;
    public static double ARM_LIMIT = 15;
    public static double EXTENT_LIMIT = 180;
    public static double ARM_KS= 10/360;


    // Robot Physical Constants
    public static double DRIVE_R = 3; // inches
    public static double DRIVE_L_GR = 3.5; // inches
    public static double DRIVE_R_GR = 8.5; // inches
    public static double DRIVE_InTk = 53./24; // inches
    public static double DRIVE_TRACT = 19; // inches
    public static double EXT__DEG_MIN = -95; // degrees
    public static double EXT__DEG_MAX = 640; // degrees
    public static double rightAngleArm= 45;// ticks

    public static double driveTolerance= 5;
    public static AHRS gyro = new AHRS();



}
