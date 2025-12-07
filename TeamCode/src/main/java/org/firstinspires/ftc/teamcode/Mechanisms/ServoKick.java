package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ConstantValues.Constants;

public class ServoKick {
    Servo kick;

    public void init(HardwareMap hwMap){
        kick = hwMap.get(Servo.class, "kick2");
        kick.setPosition(Constants.retract);
    }

    public void kick(){
        kick.setPosition(Constants.kick);
    }

    public void retract(){
        kick.setPosition(Constants.retract);
    }

    public void zeroKick() {
        kick.setPosition(0.0);
    }
}
