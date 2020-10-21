/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Collector extends SubsystemBase {
  /**
   * Creates a new Collector.
   */
  WPI_VictorSPX brushesSPX = new WPI_VictorSPX(Constants.NINJASTAR_MOTOR_CONTROLLER);
  
  WPI_VictorSPX collectorSPX = new WPI_VictorSPX(Constants.INTAKE_MOTOR_CONTROLLER);

  public Collector() {
    brushesSPX.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setBrushesMotor(double speed){
    brushesSPX.set(ControlMode.PercentOutput, speed);
  }

  public void setWheelsMotor(double speed){
    collectorSPX.set(ControlMode.PercentOutput, speed);
  }
}
