package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ConstantValues.Constants;

public class StorageConfig {

    private Servo storageServo;

    public void init(HardwareMap hwMap) {
        storageServo = hwMap.get(Servo.class , "storage");
    }

    ServoKick kick = new ServoKick();
    public void setIntakeA(){

        storageServo.setPosition(Constants.sorterIntakeA);
    }
    public void setIntakeB(){

        storageServo.setPosition(Constants.sorterIntakeB);
    }
    public void setIntakeC(){

        storageServo.setPosition(Constants.sorterIntakeC);
    }
    public void setOutA(){

        storageServo.setPosition(Constants.sorterOutTakeA);
    }
    public void setOutB(){

        storageServo.setPosition(Constants.sorterOutTakeB);
    }
    public void setOutC(){

        storageServo.setPosition(Constants.sorterOutTakeC);
    }

    public void setZero(){
        storageServo.setPosition(0.0);
    }


    public double getServoPos(){ return storageServo.getPosition();}

    public void setServo(double pos) {
        storageServo.setPosition(pos);
    }

}
