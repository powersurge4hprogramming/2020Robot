/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.data_structs;

import edu.wpi.first.wpilibj.util.Color;

/**
 * Add your docs here.
 */
public class RingBuffer {

    Color[] arr;
    int index;
    public RingBuffer(Color[] m_arr){
        arr = m_arr;
        index = 0;
    }

    public Color getIndex(int index){
        return arr[index];
    }

    public Color getNext(){
        index++;
        index = index % arr.length;
        return arr[index];

    }

    public Color getPreviousColor(){
        if(index == 0){
            index = arr.length-1;
            return arr[index];
        } else {
            index--;
            return arr[index];
        }
        
    }

    public Color getColor(){
        return arr[index];
    }

    public void setIndex(int index){
        this.index = index;
    }
}
