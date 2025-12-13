package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterConfig {
    private DcMotorEx shooter;
    private Servo hood;

    public void init(HardwareMap hwMap) {
        shooter = hwMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hood = hwMap.get(Servo.class , "hood");
    }
    public void FarAutoOut() {shooter.setVelocity(1470);}
    public void FarOut(){shooter.setVelocity(1470);}
    public void MedOut() {shooter.setVelocity(1350);}
    public void CloseOut(){shooter.setVelocity(1175);}
    public void HPIn(){
        shooter.setVelocity(-280);
    }
    public void Stop() {shooter.setVelocity(0);}
    public void hoodMed(){hood.setPosition(0.6);}
    public void hoodClose(){hood.setPosition(0.8);}
    public void hoodAutoClose(){hood.setPosition(0.84);}
    public void hoodZero(){hood.setPosition(0.0);}
    public void hoodAutoFar(){hood.setPosition(0.57);}
    public void hoodFar(){hood.setPosition(0.672);}

    public double velocityValue(){
        return shooter.getVelocity();
    }
}
