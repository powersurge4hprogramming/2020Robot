/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.data_structs.RollingAverage;
import frc.robot.data_structs.Utilities;
import frc.robot.subsystems.AimTurretSubsys;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class TeleopShootCmd extends CommandBase {
  Shooter shooter;
  AimTurretSubsys aimSubsys;
  Vision vision;
  private JoystickButton shootButton;
  private RollingAverage avgY;
  private RollingAverage avgX;

  /**
   * Creates a new TeleopShootCmd.
   */
  public TeleopShootCmd(Shooter shooter, AimTurretSubsys aimTurretSubsys, Vision vision, JoystickButton shoot) {
    this.shooter = shooter;
    this.aimSubsys = aimTurretSubsys;
    this.vision = vision;
    this.shootButton = shoot;
    avgY = new RollingAverage(32);
    avgX = new RollingAverage(32);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooter, this.aimSubsys, this.vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setPercentOutput(0);
    aimSubsys.setMotor(0);
    shooter.setAngle(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Retrieve table data and aim horizontal turret
    // Determine whether to keep angle or velocity constant, and calculate the other.
    double xAngle = avgX.sample(vision.getXOffsetAngle());
    double yAngle = avgY.sample(vision.getYOffsetAngle());
    double velocity;
    double angle;
    if(Math.tan(yAngle) == 0) {
      yAngle = 0.01;
    }
    double dist = 1.575/(Math.tan(yAngle));
    if (dist > Constants.ANGLE_VELOCITY_DIST) {
      angle = Math.max(Constants.DEFAULT_ANGLE, Utilities.getMinAngle(dist));
      velocity = Utilities.getInitialVelocity(angle, dist, Constants.HEIGHT_TO_GOAL);
    }
    else {
      velocity = Constants.DEFAULT_VELOCITY;
      angle = Utilities.getAngle(velocity, dist, Constants.HEIGHT_TO_GOAL);
      if(Double.isNaN(angle)){
        velocity = 0;
        angle = Constants.DEFAULT_ANGLE;
      }
    }
    
    // TODO: Turn the xAlign

    // TODO: Map angle to linear actuator space and apply to the turret hood
    
    // If the shoot button is held down, apply velocity to the shooter wheel
    // Map ball velocity to wheel velocity and output.
    if (shootButton.get()) {
      //velocity = ;
      shooter.setVelocityOutput(velocity);
    }
    
    shooter.setAngle(0.1);
    SmartDashboard.putNumber("Angle", angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setPercentOutput(0);
    aimSubsys.setMotor(0);
    shooter.setAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
