/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.data_structs.RingBuffer;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.WheelOfFortune;

public class PositionControl extends CommandBase {
  /**
   * Creates a new PositionControl.
   */
  private ColorSensor colorSens;
  private WheelOfFortune wheelOfFort;

  
  private Color[] colorArray;
  private RingBuffer colorBuffer;

  public PositionControl(ColorSensor m_colsens, WheelOfFortune m_wheeloff) {
    colorSens = m_colsens;
    wheelOfFort = m_wheeloff;
    // Use addRequirements() here to declare subsystem dependencies.

    /*
    colorArray = new Color[] { colorSens.kRedTarget, colorSens.kGreenTarget, 
      colorSens.kBlueTarget, colorSens.kYellowTarget };
      colorBuffer = new RingBuffer(colorArray);
      */
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int kDirection;
    String gameData;
    gameData = DriverStation.getGameSpecificMessage();
    if(gameData.length() > 0)
    {
      /*kDirection = colorSens.getDirection(gameData.charAt(0));
      wheelOfFort.setDrive(0.2*kDirection);*/
    } 
    else {
      //Code for no data received yet
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wheelOfFort.setDrive(0d);
  }

  /*
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean exit;
    if(colorSens.getDirection(DriverStation.getGameSpecificMessage().charAt(0))>0){
      exit = false;
    } else {
      exit = true;
    }
    return exit;
  }
  */
}
