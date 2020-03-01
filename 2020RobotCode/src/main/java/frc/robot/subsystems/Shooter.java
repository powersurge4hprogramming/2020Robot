/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  WPI_TalonFX talonFX = new WPI_TalonFX(Constants.SHOOTER_TALON_FX);
  WPI_VictorSPX stickyWheel = new WPI_VictorSPX(8);
  Vision vision;
  double x;
  double y;
  double theta;
  double accelertion = -9.8;
  double velocity = 0;
  double yinit = 0;
  
  public Shooter(Vision m_vis) {
    vision = m_vis;
    x = 7;
    y = 2.4;
    yinit = 0.9144;
    theta = 45;
    velocity = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double yAngle = vision.getYOffsetAngle();
    double xAngle = vision.getXOffsetAngle();
    if(Math.tan(yAngle) == 0){
      yAngle = 0.01;
    }
    x = 1.575/(Math.tan(yAngle));
    double numer = (-4.9)*(Math.pow(x, 2));
    double denom = (Math.cos(theta)*((Math.cos(theta)*y)-(Math.cos(theta)*yinit)-(Math.sin(theta)*x)));
    if(denom == 0){
      denom = -0.01;
    }
    double velocity = Math.sqrt((numer/denom));
    SmartDashboard.putNumber("Shooter Target Velocity", velocity);

  }
}
