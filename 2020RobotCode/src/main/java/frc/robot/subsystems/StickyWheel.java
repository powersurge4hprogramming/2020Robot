/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class StickyWheel extends SubsystemBase {
  WPI_VictorSPX wheel = new WPI_VictorSPX(Constants.GREENPUSHWHEEL_MOTOR_CONTROLLER);
  
  /**
   * Creates a new Feeder.
   */
  public StickyWheel() {
    wheel.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void setMotor(double speed){
    wheel.set(speed);
  }
}
