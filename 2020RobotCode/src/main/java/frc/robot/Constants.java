/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;

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
    /** 
     * Joystick axes:
     * 0 - X axis
     * 1 - Y axis
     * 2 - Z axis (rotation)
     */
    // Drive Train Motor Controllers
    public static final int BRUSH_MOTOR_CONTROLLER = 1;
    public static final int COLLECTOR_MOTOR_CONTROLLER = 2;
    public static final int FRONT_RIGHT_MOTOR_CONTROLLER = 3;
	public static final int FRONT_LEFT_MOTOR_CONTROLLER = 4;
	public static final int REAR_LEFT_MOTOR_CONTROLLER = 5;
    public static final int REAR_RIGHT_MOTOR_CONTROLLER = 6;
    public static final int STICKY_MOTOR_CONTROLLER = 8;
    

    public static final int SHOOTER_TALON_FX = 0;
    
    public static final int DRIVER_JOYSTICK_USB_PORT = 0; 
    public static final int OPERATOR_JOYSTICK_USB_PORT = 1;
    public static final int JOYSTICK_X_AXIS = 0;
    public static final int JOYSTICK_Y_AXIS = 1;
    public static final int JOYSTICK_Z_AXIS = 2;
    public static final int JOYSTICK_SCALE_AXIS = 3;
    public static final int MANUAL_TURRET_AXIS = 0;
    public static final int BUTTON_INDEX_POSITION_CONTROL = 7;
	public static final int BUTTON_INDEX_ROTATION_CONTROL = 8;
    public static final int JOYSTICK_FEEDER_BUTTON = 9;
	public static final int SET_X_ALIGN_BUTTON = 5;
	public static final int OPERATOR_COLLECT_BUTTON = 1;
    
	public static final int NUM_OF_COLOR_THRESHOLD = 10;
    public static final double Z_SCALE = 0.5;
	public static final double TOLERANCE_X_ALIGN = 1;
	public static final double ALIGN_LERP_TAU = 0.1;
	public static final double X_ALIGN_KP = 1;
	public static final double X_ALIGN_KI = 1;
	public static final double X_ALIGN_KD = 2;
    public static final double RED_VAL_FOR_BLUE = 0;
    
    public static final double HEIGHT_TO_GOAL = 1.4856;	
    public static final double ANGLE_VELOCITY_DIST = 10;
	public static final double DEFAULT_ANGLE = 45;
	public static final double DEFAULT_VELOCITY = 15;
	
}
