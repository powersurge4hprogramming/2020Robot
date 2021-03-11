// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class MoveHood extends CommandBase {
  Shooter shooter;
  RobotContainer robotContainer;
  double currentpos;
  /** Creates a new MoveHood. */
  public MoveHood(Shooter sh, RobotContainer rbot) {
    shooter = sh;
    robotContainer = rbot;
    currentpos = 0;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setAngle(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double input = robotContainer.getRawAxisOperator(Constants.OPERATOR_HOOD_AXIS)*-1;
    
    if(input < 0.015 && input > -0.015){
      input = 0;
    }
    input *= 0.001;
    currentpos = currentpos + input;
    currentpos = Math.max(currentpos, 0.2);
    currentpos = Math.min(currentpos, 0.8);

    double slider = shooter.getSliderSpeed();

    shooter.setAngle(currentpos);
  //  System.out.println(currentpos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
