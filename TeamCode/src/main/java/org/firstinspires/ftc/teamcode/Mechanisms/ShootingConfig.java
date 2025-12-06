package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShootingConfig {
    private DcMotorEx shooter;
    private Servo hood;

    public void init(HardwareMap hwMap) {
        shooter = hwMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hood = hwMap.get(Servo.class , "hood");
    }
    public void FarOut() {shooter.setVelocity(1700);}
    public void MedOut() {shooter.setVelocity(1400);}
    public void CloseOut(){shooter.setVelocity(1150);}
    public void HPIn(){
        shooter.setVelocity(-280);
    }
    public void Stop() {shooter.setVelocity(0);}
    public void hoodFar(){hood.setPosition(0.44);}
    public void hoodMed(){hood.setPosition(0.55);}
    public void hoodClose(){hood.setPosition(0.79);}
    public void hoodZero(){hood.setPosition(0.0);}
    public void hoodAutoFar(){hood.setPosition(0.48);}
    public void hoodAutoMed(){hood.setPosition(0.6);}

    public double velocityValue(){
        return shooter.getVelocity();
    }
}
