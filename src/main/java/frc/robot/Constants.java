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
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER = 0;
    public static final int AUX_CONTROLLER = 1;

    /*
   Game controller button and joystick addressing.
   Access in code by including Constants.h and using BUTTON_A
   */
    public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;
    public static final int BUTTON_X = 3;
    public static final int BUTTON_Y = 4;
    public static final int BUTTON_L_BUMP = 5;
    public static final int BUTTON_R_BUMP = 6; // Drive gear sifting
    public static final int BUTTON_SELECT = 7;
    public static final int BUTTON_START = 8;
    public static final int BUTTON_L3 = 9;
    public static final int BUTTON_R3 = 10;

    public static final int AXIS_LX = 0; // Steer left/right
    public static final int AXIS_LY = 1;
    public static final int AXIS_L_TRIG = 2; // Forward driving
    public static final int AXIS_R_TRIG = 3; // Reverse driving
    public static final int AXIS_RX = 4;
    public static final int AXIS_RY = 5;
  }

  public static class MotorConstants {
    // Falcon, Talon and Victor CAN Bus addressing
    public static final int MOTOR_RIGHT_MASTER = 1;
    public static final int MOTOR_LEFT_MASTER = 2;
    public static final int MOTOR_RIGHT_FOLLOWER = 3;
    public static final int MOTOR_LEFT_FOLLOWER = 4;
    public static final int MOTOR_Vertical_ELEVATOR = 999;// <---PLACE HOLDER!!!----------------------------

    //SparkMax turret adressing
    public static final int MOTOR_TURRET = 15;

    // Maximum velocity in units/100ms
    public static final int VELOCITY_MAX = 3200;
    public static final int VELOCITY_SP_MAX_HG = 3200;  // Maximum velocity in actual high gear
    public static final int VELOCITY_SP_MAX_LG = 1500;  // Maximum velocity in actual low gear
    public static final int VELOCITY_SP_MAX_LL = 750;   // Maximum velocity in low low virtual gear
    public static final int ELEVATOR_VELOCITY_MAX = 1000; // <---PLACE HOLDER!!!------ Maximum velocity of the elevator motor

    // Continuous current limit for Talons in amps
    public static final int CONTINUOUS_CURRENT_LIMIT = 20;
    // Peak current limit for the Talons in amps
    public static final int PEAK_CURRENT_LIMIT = 30;
    // Peak current duration for Talons in ms
    public static final int DURATION_CURRENT_LIMIT = 500;

    // PID constants PID[0] Used for low speed right side
    public static final double RIGHT_KF_0 = 0.341; // 0.0465; //0.13; // Kf = ((percent of output used for control)*1023) / (max encoder units)/100ms
    public static final double RIGHT_KP_0 = 1.4;// 0.123; // Kp = ((percent of output used for control)*1023) / Error
    public static final double RIGHT_KI_0 = 0.001;
    public static final double RIGHT_KD_0 = 0.0;

    // PID constants PID[0] Used for low speed right side
    public static final double LEFT_KF_0 = 0.341; // 0.0465; //0.13; // Kf = ((percent of output used for control)*1023) / (max encoder units)/100ms
    public static final double LEFT_KP_0 = 1.4;// 0.123; // Kp = ((percent of output used for control)*1023) / Error
    public static final double LEFT_KI_0 = 0.001;
    public static final double LEFT_KD_0 = 0;

    // PID constants PID[1] uesd for high speed left side
    public static final double RIGHT_KF_1 = 0.1598; 
    public static final double RIGHT_KP_1 = 0.0;
    public static final double RIGHT_KI_1 = 0;
    public static final double RIGHT_KD_1 = 0;

    // PID constants PID[1] uesd for high speed left side
    public static final double LEFT_KF_1 = 0.1598;
    public static final double LEFT_KP_1 = 0.0;
    public static final double LEFT_KI_1 = 0;
    public static final double LEFT_KD_1 = 0;

    // slabby encoder measurements...
    //
    // we measured moving a straight line for 80 feet
    //  encoder 1 moved -197063 ticks
    //  encoder 2 moved -197489 ticks
    //
    //  encoder 1 moved -2463.2875 ticks per foot
    //  encoder 2 moved -2468.6125 ticks per foot
    //
    //  encoder 1 moved -205.2739583 ticks per inch
    //  encoder 2 moved -205.7177083 ticks per inch
    //  
  }

  public static class MiscConstants {
    // PCM Module CAN ID
    public static final int PCM_0 = 9;

    // Solenoid Mappings for Shifter PCM
    public static final int PCM_0_GEARSHIFT = 0;  // Drive train gear shifter
    public static final int PCM_1_PREP_CONERAMP = 1; // piston used on Cone Ramp Extention

    // NavX related constants and info
    // 
    // when slabby is sitting level, depending upon which 2 sets of wheels it is sitting on,
    // pitch is: -1.18 to -0.50
    // pitch decreases (more negative) when slabby is tipping backward
    //
    public static final int NAVX_CHARGED_UP_RAMP_PITCH = 11;



    // Limelight related Constants
    public static final int LL_PIPELINE_PURPLE_CUBE_NUMBER = 8;
    public static final int LL_PIPELINE_YELLOW_CONE_NUMBER = 9;
  }
}
