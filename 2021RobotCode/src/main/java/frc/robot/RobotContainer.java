/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AimTurret;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.MechDrive;
import frc.robot.commands.TurretControl;
import frc.robot.commands.PositionControl;
import frc.robot.commands.RotationControl;
import frc.robot.commands.RunElevator;
import frc.robot.commands.RunStickyWheel;
import frc.robot.commands.SetXAlign;
import frc.robot.commands.TeleopShootCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.subsystems.AimTurretSubsys;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.DriveTrainSubsys;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StickyWheel;
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

  // Input objects
  private final Joystick m_driverInput = new Joystick(Constants.DRIVER_JOYSTICK_USB_PORT);
  private final Joystick m_operatorInput = new Joystick(Constants.OPERATOR_JOYSTICK_USB_PORT);
  private final NetworkTable m_limeTable = NetworkTableInstance.getDefault().getTable("limelight");
  
  // The subsystems
  private final DriveTrainSubsys m_driveTrain = new DriveTrainSubsys();
  private final Vision vision = new Vision(m_limeTable);
  private final ColorSensor m_colorSensor = new ColorSensor();
  private final WheelOfFortune wheelOfFortune = new WheelOfFortune();
  private final Shooter m_shooter = new Shooter(vision);
  private final StickyWheel m_stickyWheel = new StickyWheel();
  private final Collector m_collector = new Collector();
  private final AimTurretSubsys m_turret = new AimTurretSubsys(); 

  // The commands
  private final MechDrive m_mechDrive = new MechDrive(m_driveTrain, this);
  private final AimTurret m_aimCommand = new AimTurret(m_turret, this);
  private final RunElevator m_runElevator = new RunElevator(m_collector);
  private final TurretControl m_moveTurret = new TurretControl(m_shooter, this);
  private final ShootCmd m_shootcmd = new ShootCmd(m_shooter, this);

  // The autpnomous commands
  private final AutoDrive m_autoNav1 = new AutoDrive(m_driveTrain, 1);
  private final AutoDrive m_autoNav2 = new AutoDrive(m_driveTrain, 2);
  private final AutoDrive m_autoNav3 = new AutoDrive(m_driveTrain, 3);

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();






  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_driveTrain);



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //m_collector.setDefaultCommand(m_runElevator);
    m_shooter.setDefaultCommand(m_shootcmd);
    //m_turret.setDefaultCommand(m_aimCommand);
    //vision.setCameraStream(1);
  }
  public void teleopStart(){
    m_driveTrain.setDefaultCommand(m_mechDrive);
  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverInput, Constants.DRIVER_BUTTON_INDEX_ROTATION_CONTROL).whenPressed(new RotationControl(m_colorSensor, wheelOfFortune));
    new JoystickButton(m_driverInput, Constants.DRIVER_BUTTON_INDEX_POSITION_CONTROL).whenPressed(new PositionControl(m_colorSensor, wheelOfFortune));
  //  new JoystickButton(m_driverInput, 3).whileHeld(new ShootCmd(m_shooter));
    //new JoystickButton(m_driverInput, Constants.JOYSTICK_FEEDER_BUTTON).whileHeld(new FeederTestCmd(m_feeder));
    
    new JoystickButton(m_operatorInput, Constants.OPERATOR_SET_X_ALIGN_BUTTON).whileHeld(new SetXAlign(m_turret, vision, this));
    new JoystickButton(m_operatorInput, Constants.OPERATOR_COLLECT_BUTTON).whileHeld(new RunElevator(m_collector));
    //new JoystickButton(m_operatorInput, Constants.OPERATOR_AIM_BUTTON).whileHeld(new TeleopShootCmd(m_shooter, m_turret, vision));
    new JoystickButton(m_operatorInput, Constants.OPERATOR_STICKY_BUTTON).whileHeld(new RunStickyWheel(m_stickyWheel));
  }
  public double getRawAxisDriver(int axis){
    return m_driverInput.getRawAxis(axis);
  }

  public double getRawAxisOperator(int axis){
    return m_operatorInput.getRawAxis(axis);
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
    
    SmartDashboard.putData(m_chooser);
    m_chooser.setDefaultOption("Auto Nav 2", m_autoNav2);
    m_chooser.addOption("Auto Nav 1", m_autoNav1);
    m_chooser.addOption("Auto Nav 3", m_autoNav3);


    m_driveTrain.resetEncoders();
    return m_chooser.getSelected();
    //Should be an auto command
  }
}
