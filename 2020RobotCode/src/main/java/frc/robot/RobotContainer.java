/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.MechDrive;
import frc.robot.commands.PositionControl;
import frc.robot.commands.RotationControl;
import frc.robot.commands.SetXAlign;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.DriveTrainSubsys;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.WheelOfFortune;
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final DriveTrainSubsys m_driveTrain = new DriveTrainSubsys();
  private final Joystick m_driverInput = new Joystick(Constants.DRIVER_JOYSTICK_USB_PORT);
  private final MechDrive m_mechDrive = new MechDrive(m_driveTrain, this);
  private final NetworkTable m_limeTable = NetworkTableInstance.getDefault().getTable("limelight");
  private final Vision vision = new Vision(m_limeTable);
  private final ColorSensor m_colorSensor = new ColorSensor();
  private final WheelOfFortune wheelOfFortune = new WheelOfFortune();


  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_driveTrain);



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_driveTrain.setDefaultCommand(m_mechDrive);
    
    //vision.setCameraStream(1);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverInput, 1).whileHeld(new SetXAlign(m_driveTrain, vision, this));
    new JoystickButton(m_driverInput, Constants.BUTTON_INDEX_ROTATION_CONTROL).whenPressed(new RotationControl(m_colorSensor, wheelOfFortune));
    new JoystickButton(m_driverInput, Constants.BUTTON_INDEX_POSITION_CONTROL).whenPressed(new PositionControl(m_colorSensor, wheelOfFortune));
    
  }
  public double getRawAxis(int axis){
    return m_driverInput.getRawAxis(axis);
  }

  public ColorSensor getColorSensor(){
    return m_colorSensor;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
    //Should be an auto command
  }
}
