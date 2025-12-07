package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ConstantValues.Constants;
import org.firstinspires.ftc.teamcode.Mechanisms.MecDrivebase;
@TeleOp (name = "OutreachTele")
public class OutreachTeleOp extends LinearOpMode {
    MecDrivebase drive = new MecDrivebase();
    double forward, strafe, rotate;

    public void setDriver(){
        if(gamepad2.a) {
            Constants.driveOutreach = 0.3;
            forward = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;
            drive.driveFieldRelative(forward, strafe, rotate);
        }

        if(gamepad2.dpad_up) {
            drive.imu.resetYaw();
        }
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