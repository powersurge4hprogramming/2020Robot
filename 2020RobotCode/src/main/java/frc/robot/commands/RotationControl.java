/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.WheelOfFortune;

public class RotationControl extends CommandBase {
  /**
   * Creates a new RotationControl.
   */
  private ColorSensor colorSensor;
  private WheelOfFortune wheelOfFortune;

  public RotationControl(ColorSensor m_colorSensor, WheelOfFortune m_WheelOfFortune) {
    // Use addRequirements() here to declare subsystem dependencies.
    colorSensor = m_colorSensor;
    wheelOfFortune = m_WheelOfFortune;
    addRequirements(colorSensor, wheelOfFortune);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(colorSensor.getNumOfRotation() < 4){
      wheelOfFortune.setDrive(0.2);
    } else {
      wheelOfFortune.setDrive(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
