/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    // Drive Train Motor Controllers
    public static final int NINJASTAR_MOTOR_CONTROLLER = 1;
    public static final int INTAKE_MOTOR_CONTROLLER = 2;
    public static final int FRONT_RIGHT_MOTOR_CONTROLLER = 3;
	public static final int FRONT_LEFT_MOTOR_CONTROLLER = 4;
	public static final int REAR_LEFT_MOTOR_CONTROLLER = 5;
    public static final int REAR_RIGHT_MOTOR_CONTROLLER = 6;
    public static final int GREENPUSHWHEEL_MOTOR_CONTROLLER = 8;
    public static final int TURRET_MOTOR_CONTROLLER = 9;


    public static final int SHOOTER_TALON_FX = 0;


    // Wiring Ports
    public static final int LINEAR_ACTUATOR_PWM_PORT = 9;
    public static final int DRIVER_JOYSTICK_USB_PORT = 0; 
    public static final int OPERATOR_JOYSTICK_USB_PORT = 1;
    

    // Joystick Buttons
    public static final int DRIVER_BUTTON_INDEX_POSITION_CONTROL = 7; // 3D Pro Joystick
    public static final int DRIVER_BUTTON_INDEX_ROTATION_CONTROL = 8; // 3D Pro Joystick
    
    // Gamepad Buttons
	public static final int OPERATOR_SET_X_ALIGN_BUTTON = 5; // Left Bumper
    public static final int OPERATOR_COLLECT_BUTTON = 1; // A Button(?)
	public static final int OPERATOR_AIM_BUTTON = 5; //Left Bumper
	public static final int OPERATOR_SHOOT_BUTTON = 6; //Right Bumper
    public static final int OPERATOR_STICKY_BUTTON = 2; //B Button




    /** 
     * Joystick axes:
     * 0 - X axis
     * 1 - Y axis
     * 2 - Z axis (rotation)
     */
    // Joystick Axes
    public static final int DRIVER_JOYSTICK_X_AXIS = 0;
    public static final int DRIVER_JOYSTICK_Y_AXIS = 1;
    public static final int DRIVER_JOYSTICK_Z_AXIS = 2;
    public static final int DRIVER_JOYSTICK_SCALE_AXIS = 3;
    public static final int OPERATOR_JOYSTICK_COLLECTOR_AXIS = 5;
    public static final int OPERATOR_MANUAL_TURRET_AXIS = 4;
    public static final int OPERATOR_HOOD_AXIS = 1;
    

    // Color Constants
    public static final int NUM_OF_COLOR_THRESHOLD = 10;
    public static final double RED_VAL_FOR_BLUE = 0;
    

    // Turret Auto-aim constants
	public static final double LIMELIGHT_TOLERANCE_X_ALIGN = 1;
	public static final double ALIGN_LERP_TAU = 0.1;
	public static final double X_ALIGN_KP = 1;
	public static final double X_ALIGN_KI = 1;
	public static final double X_ALIGN_KD = 2;
    

    // Field Constants
    public static final double HEIGHT_TO_GOAL = 1.4856;	
    public static final double ANGLE_VELOCITY_DIST = 10;


    // Subsystem Default Constants
	public static final double DEFAULT_ANGLE = 45;
	public static final double DEFAULT_VELOCITY = 15;
	
}
