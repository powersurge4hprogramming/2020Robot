/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.data_structs.Utilities;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private NetworkTableEntry speedSlider = Shuffleboard.getTab("Shooter Speed").add("Speed", 0)
  .withWidget(BuiltInWidgets.kNumberSlider)
  .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
  .getEntry();
  WPI_TalonFX talonFX = new WPI_TalonFX(Constants.SHOOTER_TALON_FX);
  WPI_VictorSPX stickyWheel = new WPI_VictorSPX(8);
  Servo linearAct = new Servo(Constants.LINEAR_ACTUATOR_PWM_PORT);
  Vision vision;
  double x;
  double y;
  double theta;
  double accelertion = -9.8;
  double velocity = 0;
  double yinit = 0;
  double dY;
  final NeutralMode kBrakeDurNeutral = NeutralMode.Coast;
  final int kUnitsPerRevolution = 2048;
  
  public Shooter(Vision m_vis) {
    vision = m_vis;
    x = 1;
    y = 2.4;
    yinit = 0.9144;
    theta = 45;
    velocity = 0;
    dY = y-yinit;
    //linearAct.setBounds(1, 0.95, 0.5, 0.05, 0);
    TalonFXConfiguration configs = new TalonFXConfiguration();
		/* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
		configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		/* config all the settings */
    talonFX.configAllSettings(configs);
    talonFX.setNeutralMode(NeutralMode.Coast);
    talonFX.setInverted(true);
    
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
    double angle = Utilities.getAngle(25, x, dY);
    if(Double.isNaN(angle)){
      velocity = 0;
    }
    SmartDashboard.putNumber("Angle", angle);
    //System.out.println(angle);
    // Map angle to linear actuator space and assign


  }
  public void setPercentOutput(double speed){
    talonFX.set(ControlMode.PercentOutput, speed);
  }

  public void setVelocityOutput(double velocity) {
    talonFX.set(ControlMode.Velocity, velocity);
  }

  public double getVelocity(){
    double appliedMotorOutput = talonFX.getMotorOutputPercent();
		int selSenPos = talonFX.getSelectedSensorPosition(0); /* position units */
    int selSenVel = talonFX.getSelectedSensorVelocity(0); /* position units per 100ms */
    double pos_Rotations = (double) selSenPos / kUnitsPerRevolution;
		double vel_RotPerSec = (double) selSenVel / kUnitsPerRevolution * 10; /* scale per100ms to perSecond */
    double vel_RotPerMin = vel_RotPerSec * 60.0;
    double vel_MetersPerSec = vel_RotPerSec*2*Math.PI*0.0762;
    SmartDashboard.putNumber("RPM", vel_RotPerMin);
    return vel_MetersPerSec;
  }

  public void setAngle(double speed){
    linearAct.setSpeed(0.5);
    linearAct.set(speed);
  }

  public double getSliderSpeed(){
    return speedSlider.getDouble(0);
  }
}
