/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsys extends SubsystemBase {

  MecanumDrive mecanumDrive;
  // Store the mecanum drive as a field of this subsystem
  WPI_VictorSPX frontLeft = new WPI_VictorSPX(Constants.FRONT_LEFT_MOTOR_CONTROLLER);
  WPI_VictorSPX rearLeft = new WPI_VictorSPX(Constants.REAR_LEFT_MOTOR_CONTROLLER);
  WPI_VictorSPX frontRight = new WPI_VictorSPX(Constants.FRONT_RIGHT_MOTOR_CONTROLLER);
  WPI_VictorSPX rearRight = new WPI_VictorSPX(Constants.REAR_RIGHT_MOTOR_CONTROLLER);

  Encoder rightEncoder = new Encoder(6,7);
  Encoder leftEncoder = new Encoder(8,9);
  /**
   * Creates a new DriveTrainSubsys.
   * Pass the speed controllers into the subsystem constructor, to create the mecanum drive with.
   */

  public DriveTrainSubsys() {
      // Initialize the mecanum drive w/the parameters
      // MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

      leftEncoder.setReverseDirection(false);
      rightEncoder.setReverseDirection(true);
      leftEncoder.setDistancePerPulse(1.0/80.0);
      rightEncoder.setDistancePerPulse(1.0/80.0);

      frontLeft.setNeutralMode(NeutralMode.Brake);
      frontRight.setNeutralMode(NeutralMode.Brake);
      rearRight.setNeutralMode(NeutralMode.Brake);
      rearLeft.setNeutralMode(NeutralMode.Brake);

      mecanumDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   /* SmartDashboard.putData(mecanumDrive);
    SmartDashboard.putData(leftEncoder);
    SmartDashboard.putData(rightEncoder);
    SmartDashboard.putNumber("ENCODER ", leftEncoder.getDistance()); */

    SmartDashboard.putData(leftEncoder);
    SmartDashboard.putData(rightEncoder);
    
  }
  
  public void setDrive( double x, double y, double z){
    SmartDashboard.putNumber("Y FOR AUTO", y);
    mecanumDrive.driveCartesian(x, y, z);
    
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }
  public double getEncoderDist(int en) {
    switch (en) {
      case 0:
        return leftEncoder.getDistance();
        case 1:
        return rightEncoder.getDistance();
        default:
        return -1;
    }
  }
  // Add methods to set drive state
}
