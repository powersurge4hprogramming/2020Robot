/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSubsys extends SubsystemBase {

  // Store the mecanum drive as a field of this subsystem

  /**
   * Creates a new DriveTrainSubsys.
   * Pass the speed controllers into the subsystem constructor, to create the mecanum drive with.
   */
  public DriveTrainSubsys() {
      // Initialize the mecanum drive w/the parameters
      // MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  // Add methods to set drive state
}
