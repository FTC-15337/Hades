package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.ConstantValues;
import org.firstinspires.ftc.teamcode.Mechanisms.IntakeConfig;
import org.firstinspires.ftc.teamcode.Mechanisms.Kicker;
import org.firstinspires.ftc.teamcode.Mechanisms.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Mechanisms.ShootingConfig;
import org.firstinspires.ftc.teamcode.Mechanisms.StorageConfig;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends LinearOpMode {
    MecanumDrivebase drive = new MecanumDrivebase();
    IntakeConfig intake = new IntakeConfig();
    StorageConfig sorter = new StorageConfig();
    ShootingConfig shooter = new ShootingConfig();
    Kicker kick = new Kicker();

    double forward, strafe, rotate;
    public void setOperator(){
        if(gamepad2.left_trigger >= 0.7){
            intake.intakeMax();
        }else if(gamepad2.dpad_up){
            intake.reverseIntake();
        }else{
            intake.intakeStop();
        }

        if(gamepad2.a){
            sorter.setOutA();
        } else if(gamepad2.b){
            sorter.setOutB();
        } else if(gamepad2.x){
            sorter.setOutC();
        }

        if (gamepad2.y){
            kick.kick();
        } else if(gamepad2.right_bumper){
            kick.zeroKick();
        } else {
            kick.retract();
        }
    }
    public void setDriver(){
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        drive.driveFieldRelative(forward, strafe, rotate);

        if(gamepad2.right_trigger >= 0.8) {
            ConstantValues.driveMaxSpeed = 0.3;
        } else {
            ConstantValues.driveMaxSpeed = 1.0;
        }

        if(gamepad1.dpad_up){
            shooter.hoodZero();
        } else if(gamepad1.dpad_down) {
            drive.imu.resetYaw();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addLine("Initialized");
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        sorter.init(hardwareMap);
        shooter.init(hardwareMap);
        kick.init(hardwareMap);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            telemetry.addLine("OpMode is active");
            setDriver();
            setOperator();
        }
    }
}
