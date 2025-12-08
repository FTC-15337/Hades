package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ConstantValues.ConstantValues;
import org.firstinspires.ftc.teamcode.Mechanisms.IntakeConfig;
import org.firstinspires.ftc.teamcode.Mechanisms.Led;
import org.firstinspires.ftc.teamcode.Mechanisms.LimelightConfig;
import org.firstinspires.ftc.teamcode.Mechanisms.MecDrivebase;
import org.firstinspires.ftc.teamcode.Mechanisms.ShooterConfig;
import org.firstinspires.ftc.teamcode.Mechanisms.ServoKick;
import org.firstinspires.ftc.teamcode.Mechanisms.StorageConfig;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name= "TeleOp")
public class TeleOp extends LinearOpMode {
    MecDrivebase drive = new MecDrivebase();
    StorageConfig sorter = new StorageConfig();
    ShooterConfig shooter = new ShooterConfig();
    IntakeConfig intake = new IntakeConfig();
    ServoKick kick = new ServoKick();
    LimelightConfig ll = new LimelightConfig();
    Led led = new Led();
    ElapsedTime kickTimer = new ElapsedTime();
    double forward, strafe, rotate;

    public void setDriver() {
        led.startLed();
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        drive.driveFieldRelative(forward, strafe, rotate);


        if (gamepad1.right_bumper) {
            ll.alignYaw(drive);
        }

        if (gamepad1.right_trigger >= 0.8) {
            ConstantValues.driveMaxSpeed = 0.3;
        } else {
            ConstantValues.driveMaxSpeed = 1.0;
        }
        if (gamepad1.a) {
            shooter.MedOut();
            shooter.hoodMed();
        } else if (gamepad1.b) {
            shooter.FarOut();
            shooter.hoodFar();
        } else if (gamepad1.x) {
            shooter.CloseOut();
            shooter.hoodClose();
        } else if (gamepad1.left_trigger >= 0.7) {
            shooter.HPIn();
            shooter.hoodClose();
        } else {
            shooter.Stop();
        }

        if (gamepad1.dpad_up) {
            shooter.hoodZero();
        } else if (gamepad1.dpad_down) {
            drive.imu.resetYaw();
        }



    }

    static int step = -1;

    public void unsortedKick(){

        if (step == -1) return;

        switch (step) {
            case 0:
                kickTimer.reset();
                step = 1;
                break;

            case 1:
                kick.kick();
                kickTimer.reset();
                step = 2;
                break;

            case 2:
                if (kickTimer.milliseconds() > 350) {
                    kick.retract();
                    kickTimer.reset();
                    step = 3;
                }
                break;
            case 3:
                if(kickTimer.milliseconds() > 200) {
                    telemetry.addData("Sorter servo is ", sorter.getServoPos());
                    telemetry.update();
                    if(sorter.getServoPos() >= ConstantValues.sorterOutTakeA - 0.005 && sorter.getServoPos() <= ConstantValues.sorterOutTakeA + 0.005){
                        sorter.setOutC();
                    }else if(sorter.getServoPos() >= ConstantValues.sorterOutTakeC - 0.005 && sorter.getServoPos() <= ConstantValues.sorterOutTakeC + 0.005){
                        sorter.setOutB();
                    }else if(sorter.getServoPos() >= ConstantValues.sorterOutTakeB - 0.005 && sorter.getServoPos() <= ConstantValues.sorterOutTakeB + 0.005){
                        sorter.setOutA();
                    }

                    step = -1;
                }

                break;
        }


    }

    public void setOperator(){
        if(gamepad2.y && step == -1) {
            step = 0;
            kickTimer.reset();
        }

        if(gamepad2.dpad_down){
            sorter.setIntakeA();
        }

        unsortedKick();

        if(gamepad2.left_trigger >= 0.7) {
            intake.IntakeMotorMax();
            telemetry.update();
        } else {
            intake.IntakeMotorStop();
        }
        if(gamepad2.dpad_right){
            kick.kick();
        }
        if(gamepad2.dpad_left){
            kick.retract();
        }
        if(gamepad2.right_bumper){
            telemetry.update();
            telemetry.addData("Sorter Val", sorter.getServoPos());
            if(sorter.getServoPos() == ConstantValues.sorterOutTakeA){
                sorter.setOutC();
                return;
            }
            if(sorter.getServoPos() == ConstantValues.sorterOutTakeC){
                sorter.setOutB();
                return;
            }if(sorter.getServoPos() == ConstantValues.sorterOutTakeB){
                sorter.setOutA();
                return;
            }
        }

        if (gamepad2.dpad_up) {
            intake.OutIntake();
        }
        if(gamepad2.a) {
            sorter.setOutA();
        }
        if(gamepad2.b) {
            sorter.setOutB();
        }
        if(gamepad2.x) {
            sorter.setOutC();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);
        intake.init(hardwareMap);
        sorter.init(hardwareMap);
        shooter.init(hardwareMap);
        kick.init(hardwareMap);
        led.init(hardwareMap);
        ll.init(hardwareMap);

        drive.imu.resetYaw();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            setDriver();
            setOperator();

            telemetry.addData("aligned", Math.abs(ll.getTx()) < 1);

            telemetry.update();
        }
    }
}
