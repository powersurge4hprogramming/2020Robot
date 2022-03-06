/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.data_structs.RollingAverage;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class ShootCmd extends CommandBase {
  /**
   * Creates a new Test.
   */

  Shooter shooter;
  RobotContainer robotContainer;
  double currentpos;
  Vision vision;
  PIDController controller;
  private RollingAverage avg;

  public ShootCmd(Shooter m_shooter, RobotContainer m_rbot, Vision m_vision) {
    shooter = m_shooter;
    robotContainer = m_rbot;
    addRequirements(shooter);
    currentpos = 0;
    vision = m_vision;
    // turret = m_turret;
    controller = new PIDController(Constants.X_ALIGN_KP-0.35, Constants.X_ALIGN_KI, Constants.X_ALIGN_KD);
    avg = new RollingAverage(32);
    addRequirements(vision);
    SmartDashboard.setDefaultBoolean("AUTO TRACK", false);
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(0);
    controller.reset();
    // controller.setTolerance(Constants.LIMELIGHT_TOLERANCE_X_ALIGN);
    avg.reset();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double input = shooter.getSliderSpeed();
    shooter.setPercentOutput(input);

    double inputHood = robotContainer.getRawAxisOperator(Constants.OPERATOR_HOOD_AXIS) * -1;
    hoodMove(inputHood);

   
   
    double inputTurret = robotContainer.getRawAxisOperator(Constants.OPERATOR_MANUAL_TURRET_AXIS) * -0.5;
    // SmartDashboard.putNumber("Actual Velocity (m/s)", shooter.getVelocity());

    // double xOffset = avg.sample(vision.getXOffsetAngle());

    if (SmartDashboard.getBoolean("AUTO TRACK", false)) {
      double xOffset = vision.getXOffsetAngle();
      SmartDashboard.putNumber("X Offset", xOffset);
      double tolerance = Constants.LIMELIGHT_TOLERANCE_X_ALIGN;
      if (xOffset != 0 && Math.abs(xOffset) > tolerance) {
        /*
         * double tolerance = Constants.LIMELIGHT_TOLERANCE_X_ALIGN; double setZ; if
         * (Math.abs(xOffset) < tolerance) { setZ = 0; } else { double t =
         * Constants.ALIGN_LERP_TAU; setZ = xOffset * (1 - t); setZ /= 27; setZ *= 0.5;
         * if (setZ < 0) { setZ = Math.min(setZ, -0.1); } else { setZ = Math.max(setZ,
         * 0.1); }
         */
        double z = controller.calculate(xOffset);
        z /= 54;
        z = Math.max(Math.abs(z), 0.1) * -Math.signum(z);

        SmartDashboard.putNumber("Set Turret Value", z);
        shooter.moveTurret(z * -1); 

        
      }

    } else {
      turretMove(inputTurret); // IMPORTANT THIS IS WHY THE TURRET WONT MOVE
    }

  }

  private void turretMove(double input) {
    shooter.moveTurret(input);

  }

  private void hoodMove(double input) {
    if (input < 0.015 && input > -0.015) {
      input = 0;
    }
    input *= 0.003; 
    currentpos = currentpos + input;
    currentpos = Math.max(currentpos, 0.0);
    currentpos = Math.min(currentpos, 1.0);

    shooter.setActuator(currentpos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
