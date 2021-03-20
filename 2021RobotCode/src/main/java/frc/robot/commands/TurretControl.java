// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class TurretControl extends CommandBase {
  /** Creates a new MoveTurret. */
  Shooter shooter;
  RobotContainer robotContainer;
  double currentpos;

  public TurretControl(Shooter m_shooter, RobotContainer rbot) {
    shooter = m_shooter;
   robotContainer = rbot;
    addRequirements(shooter);
    currentpos = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double inputTurret = robotContainer.getRawAxisOperator(Constants.OPERATOR_MANUAL_TURRET_AXIS)*-1;
    turretMove(inputTurret);
    double inputHood = robotContainer.getRawAxisOperator(Constants.OPERATOR_HOOD_AXIS)*-1;
    hoodMove(inputHood);
  }

  private void turretMove(double input) {
    shooter.moveTurret(input);

  }
  private void hoodMove(double input) {
    if(input < 0.015 && input > -0.015){
      input = 0;
    }
    input *= 0.003;
    currentpos = currentpos + input;
    currentpos = Math.max(currentpos, 0.0);
    currentpos = Math.min(currentpos, 1.0);
    SmartDashboard.putNumber("Current Pos", currentpos);

    shooter.setActuator(currentpos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
