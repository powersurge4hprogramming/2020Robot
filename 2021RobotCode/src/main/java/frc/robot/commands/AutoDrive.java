
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.data_structs.Utilities;
import frc.robot.subsystems.DriveTrainSubsys;

public class AutoDrive extends CommandBase {
  /** Creates a new AutoDrive. */
  private DriveTrainSubsys m_driveTrain;

  private int stepNum1;
  private int stepNum2;
  private int stepNum3;

  // private boolean done;
  private double timeBetween;

  private PIDController pidY = new PIDController(0.02, 0.002, 0.035);

  private Timer time = new Timer();
  public int doCode;

  public AutoDrive(DriveTrainSubsys driveTrain, int doCode1) {
    m_driveTrain = driveTrain;
    doCode = doCode1;
    m_driveTrain.resetEncoders();
    pidY.reset();
    stepNum1 = 6;
    stepNum3 = 3;
    stepNum2 = 2;
    pidY.setTolerance(8);
    timeBetween = 3;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetEncoders();
    pidY.reset();
    stepNum1 = 6;
    stepNum2 = 3;
    stepNum3 = 6;
    pidY.setTolerance(8);
    timeBetween = 3;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putData(pidY);
    switch (doCode) {
      case 1:
        doNav1();
        break;
      case 2:
        doNav2();
        break;
      case 3:
        doNav3();
        break;
    }
  }

  private void trackDistance(double speedX, double speedY, double speedZ, int targetX, int targetY, int targetZ) {
    double sensorDist = (m_driveTrain.getEncoderDist(0) + m_driveTrain.getEncoderDist(1)) / 2;
    double sensorDistX = ((Math.abs(m_driveTrain.getEncoderDist(0)) + Math.abs(m_driveTrain.getEncoderDist(1))) / 2);
    /*
     * if (((targetY != 0 && sensorDist < targetY) || targetY == 0.0) && ((targetX
     * == 0) || (sensorDistX < targetX && targetX != 0)) && (((speedZ != 0.0) &&
     * (((sensorDistX < targetZ)))) || (speedZ == 0.0))) { // if ( sensorDist <
     * targetY) {
     * 
     * double diff = targetY - sensorDist; int slowDown = 10; int speedUp = 6; if
     * (sensorDist < speedUp) { double speedYLerped = (-1 * Math.signum(speedY)) *
     * Utilities.lerp(0.17, Math.abs(speedY), sensorDist / speedUp);
     * m_driveTrain.setDrive(speedX, speedYLerped, speedZ); } else if ((targetY -
     * sensorDist) < slowDown) { double speedYLerped = (-1 * Math.signum(speedY))
     * Utilities.lerp((-1 * Math.signum(speedY) * 0.1), Math.abs(speedY), diff /
     * slowDown); m_driveTrain.setDrive(speedX, speedYLerped, speedZ); } else {
     * m_driveTrain.setDrive(speedX, -1 * speedY, speedZ); }
     */
    // m_driveTrain.setDrive(0.0, -1*Utilities.remap(-1,1, -0.6, 0.6,
    // pidY.calculate(sensorDist, targetY)), speedZ);
    if (speedZ == 0) {
      m_driveTrain.setDrive(0.0, -1 * pidY.calculate(sensorDist, targetY),
          (m_driveTrain.getEncoderDist(1) - m_driveTrain.getEncoderDist(0)) / 100);
    } else if (speedZ != 0) {
      m_driveTrain.setDrive(0.0, 0.0, speedZ);
    }
    if ((pidY.atSetpoint() && targetY != 0) || (((targetZ != 0.0) && (((sensorDistX > targetZ)))))) {
      pidY.reset();
      // pidY.setSetpoint(0.0);
      time.reset();
      time.start();
      // m_driveTrain.setDrive(0, 0, 0);
      stepNum1--;
      stepNum2--;
      stepNum3--;
      m_driveTrain.resetEncoders();
    }

    /*
     * if ((pidY.atSetpoint()&& targetY != 0.0) || (((speedZ != 0.0) &&
     * (((sensorDistX < targetZ)))) || (speedZ == 0.0))) {
     * 
     * }
     */
  }

  private void doNav1() {
    switch (stepNum1) {
      case 6:
        trackDistance(0.0, 0.4, 0, 0, 115, 0);
        break;
      case 5:
        if (time.get() > timeBetween) {
          trackDistance(0.0, 0.0, 0.35, 0, 0, 25);
        } else {
          m_driveTrain.resetEncoders();
        }
        break;
      case 4:
        if (time.get() > timeBetween) {
          trackDistance(0.0, 0.35, 0.0, 0, 17, 0);
        } else {
          m_driveTrain.resetEncoders();
        }
        break;
      case 3:
        if (time.get() > timeBetween) {
          trackDistance(0.0, 0.0, 0.35, 0, 0, 25);
        } else {
          m_driveTrain.resetEncoders();
        }
        break;
      case 2:
        if (time.get() > timeBetween) {
          trackDistance(0.0, 0.35, 0.0, 0, 20, 0);
        } else {
          m_driveTrain.resetEncoders();
        }
        break;
      case 1:
        if (time.get() > timeBetween) {
          trackDistance(0.0, 0.35, 0.0, 0, 84, 0);
        } else {
          m_driveTrain.resetEncoders();
        }
        break;
      case -1:
        if (time.get() > timeBetween) {
          trackDistance(0.0, 0.35, 0.0, 0, 84, 0);
        } else {
          m_driveTrain.resetEncoders();
        }
        break;
      case 0:
        m_driveTrain.setDrive(0, 0, 0);
        m_driveTrain.resetEncoders();
        break;
      default:
        m_driveTrain.setDrive(0, 0, 0);
        m_driveTrain.resetEncoders();
    }
  }

  private void doNav2() {
    SmartDashboard.putNumber("Step Num", stepNum2);
    switch (stepNum2) {
      case 3:
        trackDistance(0.0, 0.0, 0.0, 0, 60, 0);
        break;
      case 2:
        if (time.get() > timeBetween) {
          trackDistance(0.0, 0.0, 0.5, 0, 0, 22);
        } else  {
          m_driveTrain.resetEncoders();
        }
        break;
      case 1:
        if (time.get() > timeBetween) {
          trackDistance(0.0, 0.0, 0.0, 0, 60, 0);
        } else  {
          m_driveTrain.resetEncoders();
        }
        break;
      case 0:
        m_driveTrain.setDrive(0, 0, 0);
        m_driveTrain.resetEncoders();
        break;
      default:
        m_driveTrain.setDrive(0, 0, 0);
        m_driveTrain.resetEncoders();
    }
  }

  private void doNav3() {
    switch (stepNum3) {
      case 6:
        trackDistance(0.0, 0.35, 0, 0, 60, 0);
        break;
      case 5:
        if (time.get() > timeBetween) {
          trackDistance(0.0, 0.0, -0.35, 0, 0, 25);
        } else {
          m_driveTrain.resetEncoders();
        }
        break;
      case 4:
        if (time.get() > timeBetween) {
          trackDistance(0.0, 0.35, 0.0, 0, 60, 0);
        } else {
          m_driveTrain.resetEncoders();
        }
        break;
      case 3:
        if (time.get() > timeBetween) {
          trackDistance(0.0, -0.35, 0, 0, 60, 0);
        } else {
          m_driveTrain.resetEncoders();
        }
        break;
      case 2:
        if (time.get() > timeBetween) {
          trackDistance(0.0, 0.0, -0.4, 0, 0, 25);
        } else {
          m_driveTrain.resetEncoders();
        }
        break;
      case 1:
        if (time.get() > timeBetween) {
          trackDistance(0.0, 0.35, 0.0, 0, 60, 0);
        } else {
          m_driveTrain.resetEncoders();
        }
        break;
      case 0:
        m_driveTrain.setDrive(0, 0, 0);
        m_driveTrain.resetEncoders();
        break;
      default:
        m_driveTrain.setDrive(0, 0, 0);
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