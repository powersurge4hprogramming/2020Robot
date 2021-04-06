/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsys;

public class MechDrive extends CommandBase {
  private DriveTrainSubsys driveTrain;
  private RobotContainer robotContainer;
  /**
   * Creates a new MechDrive.
   * driveTrain feild is null 
   * driveTrain parameter is passed form Robot Container
   */
  public MechDrive(DriveTrainSubsys driveTrain, RobotContainer robocont) {
    //sets created DriveTrainSubsystem to one that was imported
    this.driveTrain = driveTrain;
    this.robotContainer = robocont;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = robotContainer.getRawAxisDriver(Constants.DRIVER_JOYSTICK_X_AXIS);
    double y = robotContainer.getRawAxisDriver(Constants.DRIVER_JOYSTICK_Y_AXIS);
    double z = robotContainer.getRawAxisDriver(Constants.DRIVER_JOYSTICK_Z_AXIS);
    double scale = robotContainer.getRawAxisDriver(Constants.DRIVER_JOYSTICK_SCALE_AXIS);
    //Formula for scale (1-x)/2
    scale = (1-scale)/2;
    //squares inputs
    x = squareInput(x)*scale;
    y = squareInput(y)*scale;
    //Sqaures z, the scale, then applies to z
    z = squareInput(z) * scale;
    
    driveTrain.setDrive(x, y, z);

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    if(interrupted){
      driveTrain.setDrive(0, 0, 0);
    } else {
      driveTrain.setDrive(0, 0, 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public double squareInput(double input){
    return Math.pow(input, 2) * Math.signum(input);
  }
}
