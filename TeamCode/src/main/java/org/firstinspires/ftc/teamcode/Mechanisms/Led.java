package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Led {
    private Servo led;

    public void init(HardwareMap hwMap) {
        led = hwMap.get(Servo.class, "led");

    }
    public void redLed(){
        led.setPosition(0.28);
    }

    public void yellowLed(){
        led.setPosition(0.388);
    }
    public void stopLed(){
        led.setPosition(0.0);
    }
}


