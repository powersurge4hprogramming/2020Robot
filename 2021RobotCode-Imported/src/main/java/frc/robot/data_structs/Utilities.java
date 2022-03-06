/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.data_structs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class Utilities {
    public static final double getInitialVelocity(double theta, double x, double dY){
        double radians = Math.toRadians(theta);
        double g = -9.81;
        double num = g*Math.pow(x, 2);
        double denom = (x*Math.sin(2*radians)) - (2*dY*Math.pow(Math.cos(radians), 2));
        if(denom < 0){
            return Math.sqrt(num/denom);
        }
        return Double.NaN;
    }

    public static final double lerp(double a, double b, double tau) {
        tau = Math.max(tau, 0.0);
        tau = Math.min(tau, 1.0);
        return (b*tau)+(a*(1-tau));
    }
    public static final double invLerp(double a, double b, double val) {
      /*  val = Math.max(val, 0.2);
        val = Math.min(val, 0.8); */
        return (val - a) / (b - a);
    }
    public static final double remap(double oldA, double oldB, double newA, double newB, double value) {
        double result = invLerp(oldA, oldB, value);
        return lerp(newA, newB, result);
    }


    public static final double getMinAngle(double dist) {
        double result = 0;
        double h = Constants.HEIGHT_TO_GOAL;
        result = Math.toDegrees(Math.atan(h / dist));
        return result;
    }

    public static final double getAngle(double initialVelocity, double x, double dY){
        double g = -9.81;
        double squareRoot = (Math.pow(initialVelocity, 4)) - g*(g*Math.pow(x, 2)+2*dY*Math.pow(initialVelocity, 2));
        //System.out.println("SquareRoot " + squareRoot);
        if(squareRoot < 0){
            return Double.NaN;
        }
        
        double numPlus = Math.pow(initialVelocity, 2) + Math.sqrt(squareRoot);
        //System.out.println("Num Plus "+numPlus);
        double numNegative = Math.pow(initialVelocity, 2) - Math.sqrt(squareRoot);
        //System.out.println(numNegative);
        double denom = g*x;
        //System.out.println(denom);
        double answerPlus = Math.atan(numPlus/denom);
        answerPlus = Math.toDegrees(answerPlus);
        //System.out.println(answerPlus);
        double answerNegative = Math.atan(numNegative/denom);
        answerNegative = Math.toDegrees(answerNegative);
        return Math.max(answerPlus, answerNegative);
    }
}