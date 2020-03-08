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
import frc.robot.data_structs.RollingAverage;
import frc.robot.subsystems.AimTurretSubsys;
import frc.robot.subsystems.DriveTrainSubsys;
import frc.robot.subsystems.Vision;

public class SetXAlign extends CommandBase {
  /**
   * Creates a new SetXAlign.
   */
  Vision vision;
  PIDController controller;
  RobotContainer robotContainer;
  private RollingAverage avg;
  AimTurretSubsys turret;

  public SetXAlign(AimTurretSubsys m_turret, Vision m_vision, RobotContainer m_robocont) {
    // Use addRequirements() here to declare subsystem dependencies.
    vision = m_vision;
    turret = m_turret;
    robotContainer = m_robocont;
    controller = new PIDController(Constants.X_ALIGN_KP, Constants.X_ALIGN_KI, Constants.X_ALIGN_KD);
    avg = new RollingAverage(32);
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(0);
    controller.reset();
    controller.setTolerance(Constants.TOLERANCE_X_ALIGN);
    avg.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xOffset = avg.sample(vision.getXOffsetAngle());
    double tolerance = Constants.TOLERANCE_X_ALIGN;
    double setZ;
    if(Math.abs(xOffset) < tolerance) {
      setZ=0;
    } else {
      double t = Constants.ALIGN_LERP_TAU;
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
    SmartDashboard.putNumber("Set Turret Value", setZ);
    turret.setMotor(setZ);
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
    }
    */
    return false;
  }
}
