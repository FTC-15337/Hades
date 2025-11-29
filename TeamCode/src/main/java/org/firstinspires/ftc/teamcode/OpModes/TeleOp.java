package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.ConstantValues;
import org.firstinspires.ftc.teamcode.Mechanisms.IntakeConfig;
import org.firstinspires.ftc.teamcode.Mechanisms.MecanumDrivebase;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends LinearOpMode {
    MecanumDrivebase drive = new MecanumDrivebase();
    IntakeConfig intake = new IntakeConfig();

    double forward, strafe, rotate;
    public void setOperator(){
        if(gamepad2.left_trigger > 0.7){
            intake.intakeMax();
        }else if(gamepad2.dpad_up){
            intake.reverseIntake();
        }else{
            intake.intakeStop();
        }
    }
    public void setDriver(){
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        drive.driveFieldRelative(forward, strafe, rotate);

        if(gamepad1.right_trigger >= 0.8) {
            ConstantValues.driveMaxSpeed = 0.3;
        } else {
            ConstantValues.driveMaxSpeed = 1.0;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addLine("Initialized");
        intake.init(hardwareMap);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            telemetry.addLine("OpMode is active");
            setOperator();
            setDriver();
        }
    }

}
