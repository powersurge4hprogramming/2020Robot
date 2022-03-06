/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.data_structs.RingBuffer;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.WheelOfFortune;

public class RotationControl extends CommandBase {
  /**
   * Creates a new RotationControl.
   */
  private ColorSensor colorSensor;
  private WheelOfFortune wheelOfFortune;

  private Color[] colorArray;
  private RingBuffer ringBuffer;
  private int numOfChange;
  private int numOfRotations;
  private String previousColor;
  private int colorCount;

  public RotationControl(ColorSensor m_colorSensor, WheelOfFortune m_WheelOfFortune) {
        // Use addRequirements() here to declare subsystem dependencies.
    colorSensor = m_colorSensor;
    wheelOfFortune = m_WheelOfFortune;
    addRequirements(colorSensor, wheelOfFortune);
/*
    colorArray = new Color[] { colorSensor.kRedTarget, colorSensor.kGreenTarget, 
                                colorSensor.kBlueTarget, colorSensor.kYellowTarget };
    ringBuffer = new RingBuffer(colorArray);

    numOfChange = -1;
    numOfRotations = 0;
    previousColor = "";
    colorCount = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorSensor.reset();
    this.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    ColorMatchResult match = colorSensor.getColorMatch();
    if(ringBuffer.getColor().equals(match.color)){
      colorCount++;
    } else {
      colorCount = 0;
    }

    if(colorCount >= Constants.NUM_OF_COLOR_THRESHOLD){
      ringBuffer.getNext();
      numOfChange++;
      colorCount = 0;
    }

    numOfRotations = numOfChange / 8;
    previousColor = colorSensor.getColorString(match);
    if(numOfRotations < 4) {
      wheelOfFortune.setDrive(0.5);
    } else {
      wheelOfFortune.setDrive(0);
    }
*/
    printAll();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wheelOfFortune.setDrive(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean exit = (numOfRotations >= 4);
    return exit;
  }

  public void reset(){
    //this is a comment
    /*
    numOfChange = -1;
    numOfRotations = 0;
    previousColor = "";
    colorCount = 0;
    ColorMatchResult detectedColor = colorSensor.getColorMatch();
    while(!ringBuffer.getColor().equals(detectedColor.color)){
      ringBuffer.getNext();
    }
    previousColor = colorSensor.getColorString(ringBuffer.getColor());
    */
  }

  public void printAll() {
   /* SmartDashboard.putNumber("Number Of Changes", numOfChange);
    SmartDashboard.putNumber("Number Of Rotations", numOfRotations);
    SmartDashboard.putNumber("Color Count", colorCount);
    SmartDashboard.putString("Expected Color", colorSensor.getColorString(ringBuffer.getColor())); */
  }
}
