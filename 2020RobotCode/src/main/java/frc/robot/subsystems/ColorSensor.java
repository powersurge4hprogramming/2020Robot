/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
  /**
   * Creates a new ColorSensor.
  */
  
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private ColorMatch colorMatcher = new ColorMatch();
  ColorMatchResult match;
  public final Color kBlueTarget = ColorMatch.makeColor(0.113, 0.410, 0.472);
  public final Color kGreenTarget = ColorMatch.makeColor(0.167, 0.591, 0.240);
  public final Color kRedTarget = ColorMatch.makeColor(0.549, 0.333, 0.114);
  public final Color kYellowTarget = ColorMatch.makeColor(0.320, 0.572, 0.100);
  //private Color[] colorArray = {kRedTarget, kGreenTarget, kBlueTarget, kYellowTarget};
  // Move the Color objects into constants, so we don't need 3 vars per color.
  //private final Color kBlueTarget = ColorMatch.makeColor(Constants.RED_VAL_FOR_BLUE, Constants.RED_VAL_FOR_BLUE, b)
  public ColorSensor() {
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget); 
    match = colorMatcher.matchClosestColor(colorSensor.getColor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public ColorMatchResult getColorMatch() {
    Color detectedColor = colorSensor.getColor();
    double IR = colorSensor.getIR();
    int proximity = colorSensor.getProximity();
    String colorString = getColorString(detectedColor);
    printAll(detectedColor, IR, proximity, colorString);
    return colorMatcher.matchClosestColor(detectedColor);
  }

  public String getColorString(ColorMatchResult match){
    String colorString;
    colorString = getColorString(match.color);
    return colorString;
  }

  public String getColorString(Color match){
    String colorString;
    if (match == kBlueTarget) {
      colorString = "Blue";
    } else if (match == kRedTarget) {
      colorString = "Red";
    } else if (match == kGreenTarget) {
      colorString = "Green";
    } else if (match == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    return colorString;
  }

  public void reset() {
    Color detectedColor = colorSensor.getColor();
    match = colorMatcher.matchClosestColor(detectedColor);
  }

  /**
   * color: char from FMS - describes the TARGET color.
   */
  public int getDirection(char color){
    int kDirection;
    Color detectedColor = colorSensor.getColor();
    String currentColor = getColorString(colorMatcher.matchClosestColor(detectedColor));
    //Translates desired color into color need to align with field color sensor
    if(color == 'B'){
      color = 'R';
    } else if(color == 'G'){
      color = 'Y';
    } else if(color == 'Y'){
      color = 'G';
    } else if(color == 'R'){
      color = 'B';
    }
    //If its not at the color than keep driving, if it is then stop
    if(currentColor.charAt(0) == (color)){
      kDirection = 0;
    } else {
      kDirection = 1;
    }
    return kDirection ;
  }


  public void printAll(Color detectedColor, double IR, int proximity, String colorString){
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putNumber("Proximity", proximity);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }
}
