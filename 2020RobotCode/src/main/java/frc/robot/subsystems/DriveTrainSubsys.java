/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
  /**
   * Creates a new DriveTrainSubsys.
   * Pass the speed controllers into the subsystem constructor, to create the mecanum drive with.
   */

  public DriveTrainSubsys() {
      // Initialize the mecanum drive w/the parameters
      // MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
      mecanumDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData(mecanumDrive);
    
    
  }
  
  public void setDrive( double x, double y, double z){
    mecanumDrive.driveCartesian(x, y, z);
    
  }
  // Add methods to set drive state
}
