package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.ConstantValues;
import org.firstinspires.ftc.teamcode.Mechanisms.MecanumDrivebase;

import java.net.ContentHandler;

@TeleOp (name = "OutreachTele")
public class OutreachTeleOp extends LinearOpMode {
    MecanumDrivebase drive = new MecanumDrivebase();
    double forward, strafe, rotate;

    public void setDriver(){
        ConstantValues.driveMaxSpeed = ConstantValues.outreachDriveSpeed;
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        drive.driveFieldRelative(forward, strafe, rotate);
    }
    @Override
    public void runOpMode(){
        drive.init(hardwareMap);
        telemetry.addLine("Initialized");
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            telemetry.addLine("OpMode is active");
            setDriver();
            telemetry.update();
        }
    }
}