package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeConfig {
    private DcMotor intakeL;
    private DcMotor intakeR;
    public void init(HardwareMap hwMap){
        intakeL = hwMap.get(DcMotor.class , "intakeMotorR");
        intakeR = hwMap.get(DcMotor.class, "intakeMotorL");
        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeR.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intakeMax(){
        intakeL.setPower(ConstantValues.intakeMax);
        intakeR.setPower(ConstantValues.intakeMax);
    }
    public void intakeStop(){
        intakeL.setPower(ConstantValues.intakeStop);
        intakeR.setPower(ConstantValues.intakeStop);
    }
    public void reverseIntake(){
        intakeL.setPower(ConstantValues.intakeReverse);
        intakeL.setPower(ConstantValues.intakeReverse);
    }
}

