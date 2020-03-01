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

    private LinkedList<Float> entryQueue;
    private float sum;
    private int maxSize;

    public RollingAverage(int size) {
        entryQueue = new LinkedList<>();
        maxSize = Math.max(0, size);
    }
    
    public float getAvg() {
        return sum / entryQueue.size();
    }

    public float sample(float sample) {
        entryQueue.add(sample);
        resize();
        return getAvg();
    }


    public float setSize(int size) {
        this.maxSize = Math.max(0, size);
        resize();
        return getAvg();
    }

    private void resize() {
        while(entryQueue.size() > maxSize) {
            sum -= entryQueue.remove();
        }
    }
}
