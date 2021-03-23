  
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.data_structs.Utilities;
import frc.robot.subsystems.DriveTrainSubsys;

public class AutoDrive extends CommandBase {
  /** Creates a new AutoDrive. */
  private DriveTrainSubsys m_driveTrain;

  private int stepNum;
 // private boolean done;
  private double timeBetween;

  private Timer time = new Timer();

  public AutoDrive(DriveTrainSubsys driveTrain) {
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stepNum = 3;
    m_driveTrain.resetEncoders();
 //   done = false;
    timeBetween = 1.5;
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
        trackDistance(0.0, 0.35, 0, 0, 60);
        break;
      case 2:
      if (time.get()>timeBetween) {
       // done = false;
        trackDistance(0.0, -0.35, 0, 0, 60);
     //   SmartDashboard.putBoolean("Done?", done);
      }
        break;
      case 1:
      if (time.get()>timeBetween) {
    //    done = false;
      //  SmartDashboard.putBoolean("Done?", done);
        trackDistance(0.0, 0.35, 0.0, 0, 60);
      }
        break;
      case 0:
        m_driveTrain.setDrive(0, 0, 0);
        break;
      default:
        m_driveTrain.setDrive(0, 0, 0);
    }
  }

  private void trackDistance(double speedX, double speedY, double speedZ, int targetX, int targetY) {
    double sensorDist = Math.abs((m_driveTrain.getEncoderDist(0)+m_driveTrain.getEncoderDist(1))/2);
    double sensorDistX = ((Math.abs(m_driveTrain.getEncoderDist(0))+Math.abs(m_driveTrain.getEncoderDist(1)))/2);
  //  if (((targetY != 0 && sensorDist < targetY) || targetY == 0) && sensorDistX < targetX) {
      if ( sensorDist < targetY) {

      double diff = targetY - sensorDist;
      int slowDown = 14;
      int speedUp = 14;
      if (sensorDist<speedUp) {
        double speedYLerped =  (-1*Math.signum(speedY)) * Utilities.lerp(0.15, Math.abs(speedY), sensorDist / speedUp);
        m_driveTrain.setDrive(speedX, speedYLerped, speedZ);
      }
      else if ((targetY-sensorDist)<slowDown) {
        double speedYLerped = (-1*Math.signum(speedY)) * Utilities.lerp((-1*Math.signum(speedY)*0.2), Math.abs(speedY), diff / slowDown);
        m_driveTrain.setDrive(speedX, speedYLerped, speedZ);
      } else {
        m_driveTrain.setDrive(speedX, -1 * speedY, speedZ);
      }
    } else {
      time.reset();
      time.start();
   //   done = true;
     // SmartDashboard.putBoolean("Done?", done);
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