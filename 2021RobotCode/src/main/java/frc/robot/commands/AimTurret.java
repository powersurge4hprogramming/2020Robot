/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AimTurretSubsys;

public class AimTurret extends CommandBase {
  /**
   * Creates a new AimTurret.
   */
AimTurretSubsys turret;
private RobotContainer robotContainer;

  public AimTurret(AimTurretSubsys m_turret, RobotContainer robocont) {
    // Use addRequirements() here to declare subsystem dependencies.
    turret = m_turret;
    this.robotContainer = robocont;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.setMotor(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double turnPercent = robotContainer.getRawAxisOperator(Constants.OPERATOR_MANUAL_TURRET_AXIS);
    //turnPercent = Math.pow(turnPercent, 2) * Math.signum(turnPercent);

    //turret.setMotor(turnPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
