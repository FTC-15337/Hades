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
    public void FarOut() {shooter.setVelocity(1580);}
    public void MedOut() {shooter.setVelocity(1350);}
    public void CloseOut(){shooter.setVelocity(1175);}
    public void HPIn(){
        shooter.setVelocity(-280);
    }
    public void Stop() {shooter.setVelocity(0);}
    public void hoodFar(){hood.setPosition(0.465);}
    public void hoodMed(){hood.setPosition(0.55);}
    public void hoodClose(){hood.setPosition(0.67);}
    public void hoodZero(){hood.setPosition(0.0);}
    public void OutAutoFar(){shooter.setVelocity(1600);}
    public void hoodAutoClose(){hood.setPosition(0.64);}

    public double velocityValue(){
        return shooter.getVelocity();
    }
}
