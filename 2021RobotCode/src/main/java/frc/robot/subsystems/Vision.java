/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
public class Vision extends SubsystemBase {
  /**
   * Creates a new Vision.
   */

   //Creates used limelight table object
   private NetworkTable limeTable;
  //Defines values
  NetworkTableEntry tx; 
  NetworkTableEntry ty;
  NetworkTableEntry ta;

  

  //post to smart dashboard periodically
  
  public Vision(NetworkTable m_limeTable) {
    limeTable = m_limeTable;
    tx = limeTable.getEntry("tx");
    ty = limeTable.getEntry("ty");
    ta = limeTable.getEntry("ta");
  }

  

  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    //Read of limelight values periodically and posts to dashboard
    
    SmartDashboard.putNumber("LimelightX", getXOffsetAngle());
    SmartDashboard.putNumber("LimelightY", getYOffsetAngle());
    SmartDashboard.putNumber("LimelightArea", getPercentArea());
  }

  public double getXOffsetAngle(){
    double x = tx.getDouble(0.0);
    return x;
  }
  public double getYOffsetAngle(){
    double y = ty.getDouble(0.0);
    return y;
  }
  public double getPercentArea(){
    double area = ta.getDouble(0.0);
    return area;
  }
  /*
  Acceptable inputs: 0,1,2 
  0 = Standard - Side-by-side streams if a webcam is attached to Limelight
  1 = PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
  2 = PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
  */
  public void setCameraStream(int mode){
    if(mode == 0){
      limeTable.getEntry("stream").setNumber(0);
    } else if(mode == 1){
      limeTable.getEntry("stream").setNumber(1);
    } else if(mode == 2){
      limeTable.getEntry("stream").setNumber(2);
    } else {
      //Do nothing
    }
  }
}
