// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.data_structs.Utilities;
import frc.robot.subsystems.DriveTrainSubsys;

public class AutoDrive extends CommandBase {
  /** Creates a new AutoDrive. */
  private DriveTrainSubsys m_driveTrain;

  private int stepNum;
  private int direction;

  public AutoDrive(DriveTrainSubsys driveTrain) {
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stepNum = 3;
    m_driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (stepNum) {
      case 7:
        break;
      case 6:
        break;
      case 5:
        break;
      case 4:
        break;
      case 3:
        trackDistance(0, 0.4, 60);
        break;
      case 2:
        trackDistance(0, -0.4, 60);
        break;
      case 1:
        trackDistance(0, 0.4, 60);
        break;
      case 0:
        m_driveTrain.setDrive(0, 0, 0);
        break;
      default:
        m_driveTrain.setDrive(0, 0, 0);
    }
  }

  private void trackDistance(double speedX, double speedY, int targetY) {
    if (Math.abs(m_driveTrain.getEncoderDist(0)) < targetY) {
      double diff = targetY - Math.abs(m_driveTrain.getEncoderDist(0));
      int slowDown = 14;
      int speedUp = 14;
      if (Math.abs(m_driveTrain.getEncoderDist(0))<speedUp) {
        double speedYLerped =  (-1*Math.signum(speedY)) * Utilities.lerp(0.15, Math.abs(speedY), (Math.abs(m_driveTrain.getEncoderDist(0) / speedUp)));
        m_driveTrain.setDrive(speedX, speedYLerped, 0);
      }
      else if ((targetY-Math.abs(m_driveTrain.getEncoderDist(0)))<slowDown) {
        double speedYLerped = (-1*Math.signum(speedY)) * Utilities.lerp(0.0, Math.abs(speedY), diff / slowDown);
        m_driveTrain.setDrive(speedX,speedYLerped, 0);
      } else {
        m_driveTrain.setDrive(speedX, -1 * speedY, 0);
      }
    } else {
      m_driveTrain.setDrive(0, 0, 0);
      stepNum--;
      m_driveTrain.resetEncoders();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.setDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
