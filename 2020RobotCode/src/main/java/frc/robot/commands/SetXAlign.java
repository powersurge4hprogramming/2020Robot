/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsys;
import frc.robot.subsystems.Vision;

public class SetXAlign extends CommandBase {
  /**
   * Creates a new SetXAlign.
   */
  DriveTrainSubsys driveTrain;
  Vision vision;
  PIDController controller;
  RobotContainer robotContainer;

  public SetXAlign(DriveTrainSubsys m_driveTrain, Vision m_vision, RobotContainer m_robocont) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = m_driveTrain;
    vision = m_vision;
    robotContainer = m_robocont;
    controller = new PIDController(Constants.X_ALIGN_KP, Constants.X_ALIGN_KI, Constants.X_ALIGN_KD);
    addRequirements(driveTrain);
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(0);
    controller.reset();
    controller.setTolerance(Constants.TOLERANCE_X_ALIGN);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xOffset = vision.getXOffsetAngle();
    double tolerance = Constants.TOLERANCE_X_ALIGN;
    double setZ;
    if(Math.abs(xOffset) < tolerance){
      setZ=0;
    } else {
      double t = Constants.ALIGN_LERP_T;
      setZ = xOffset*(1-t);
      setZ /= 27;
      setZ*=0.5;
      if(setZ<0){
        setZ = Math.min(setZ, -0.1);
      } else {
        setZ = Math.max(setZ, 0.1);
      }
      
    }
    
    
    double z = controller.calculate(xOffset);
    z /= 54;
    z = Math.max(Math.abs(z), 0.1) * -Math.signum(z);
    double x = robotContainer.getRawAxis(Constants.JOYSTICK_X_AXIS);
    double y = robotContainer.getRawAxis(Constants.JOYSTICK_Y_AXIS);
    y*=-1;
    SmartDashboard.putNumber("Set Z Val", setZ);
    driveTrain.setDrive(x, y, setZ);
    //driveTrain.setDrive(0, 0, z);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*double xOffset = vision.getXOffsetAngle();
    double tolerance = Constants.TOLERANCE_X_ALIGN;
    if(Math.abs(xOffset) < tolerance){
      return true;
    }*/
    return false;
  }
}
