/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.data_structs;

import java.util.LinkedList;

/**
 * Add your docs here.
 */
public class RollingAverage {

    private LinkedList<Double> entryQueue;
    private double sum;
    private int maxSize;

    public RollingAverage(int size) {
        entryQueue = new LinkedList<>();
        sum = 0;
        maxSize = Math.max(0, size);
    }
    
    public double getAvg() {
        return sum / entryQueue.size();
    }

    public double sample(double sample) {
        entryQueue.add(sample);
        resize();
        return getAvg();
    }


    public double setSize(int size) {
        this.maxSize = Math.max(0, size);
        resize();
        return getAvg();
    }

    public void reset() {
        entryQueue = new LinkedList<>();
        sum = 0;
    }

    private void resize() {
        while(entryQueue.size() > maxSize && entryQueue.size() > 0) {
            sum -= entryQueue.remove();
        }
    }
}
