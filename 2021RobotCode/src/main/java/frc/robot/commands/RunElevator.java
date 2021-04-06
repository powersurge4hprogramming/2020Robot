/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Collector;

public class RunElevator extends CommandBase {
  /**
   * Creates a new Collect.
   */
  private RobotContainer robotContainer;
  Collector collector;
  public RunElevator(Collector m_collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    collector = m_collector;
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collector.setBrushesMotor(0);
    //collector.setWheelsMotor(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*double input = robotContainer.getRawAxisOperator(Constants.OPERATOR_JOYSTICK_COLLECTOR_AXIS);
    input = input*input*Math.signum(input)*0.7*-1;
    double slider = collector.getSpeedSlider(0);
    collector.setBrushesMotor(slider);
    collector.setWheelsMotor(0);
    */
    collector.setBrushesMotor(0.8);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.setBrushesMotor(0);
    //collector.setWheelsMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
