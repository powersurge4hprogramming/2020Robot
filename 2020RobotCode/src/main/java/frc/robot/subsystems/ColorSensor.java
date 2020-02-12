/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ColorSensor extends SubsystemBase {
  /**
   * Creates a new ColorSensor.
  */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private ColorMatch colorMatcher = new ColorMatch();
  // Move the Color objects into constants, so we don't need 3 vars per color.
  //private final Color kBlueTarget = ColorMatch.makeColor(Constants.RED_VAL_FOR_BLUE, Constants.RED_VAL_FOR_BLUE, b)
  public ColorSensor() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
