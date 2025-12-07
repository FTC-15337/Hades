package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Led {
    private Servo led;

    public void init(HardwareMap hwMap) {
        led = hwMap.get(Servo.class, "led");

    }
    public void startLed(){
        led.setPosition(1.0);
    }
}


