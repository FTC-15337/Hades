package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ConstantValues.Constants;
import org.firstinspires.ftc.teamcode.Mechanisms.IntakeConfig;
import org.firstinspires.ftc.teamcode.Mechanisms.Led;
import org.firstinspires.ftc.teamcode.Mechanisms.LimelightConfig;
import org.firstinspires.ftc.teamcode.Mechanisms.MecDrivebase;
import org.firstinspires.ftc.teamcode.Mechanisms.ShooterConfig;
import org.firstinspires.ftc.teamcode.Mechanisms.ServoKick;
import org.firstinspires.ftc.teamcode.Mechanisms.SortingWithColor;
import org.firstinspires.ftc.teamcode.Mechanisms.StorageConfig;

import java.util.Timer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name= "TeleOp")
public class TeleOp extends LinearOpMode {
    MecDrivebase drive = new MecDrivebase();
    StorageConfig sorter = new StorageConfig();
    //SortingWithColor colorSensor = new SortingWithColor();
    ShooterConfig shooter = new ShooterConfig();
    IntakeConfig intake = new IntakeConfig();
    ServoKick kick = new ServoKick();
    LimelightConfig ll = new LimelightConfig();
    Led led = new Led();
    ElapsedTime kickTimer = new ElapsedTime();
    ElapsedTime pathTimer = new ElapsedTime();
    double servoValue;
    double forward, strafe, rotate;
    static double[][] sortingValues;
    static double[][] storedValues;
    double velocity;

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
            Constants.driveMaxSpeed = 0.3;
        } else {
            Constants.driveMaxSpeed = 1.0;
        }
        if (gamepad1.a) {
            shooter.MedOut();
            shooter.hoodMed();
            ////////////velocity = 1350;
        } else if (gamepad1.b) {
            shooter.FarOut();
            shooter.hoodFar();
            ////////////velocity = 1700;
        } else if (gamepad1.x) {
            shooter.CloseOut();
            shooter.hoodClose();
            ////////////velocity = 1150;
        } else if (gamepad1.left_trigger >= 0.7) {
            shooter.HPIn();
            shooter.hoodClose();
        } else {
            shooter.Stop();
            ////////////velocity = 0;
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
                    if(sorter.getServoPos() >= Constants.sorterOutTakeA - 0.005 && sorter.getServoPos() <= Constants.sorterOutTakeA + 0.005){
                        sorter.setOutC();
                    }else if(sorter.getServoPos() >= Constants.sorterOutTakeC - 0.005 && sorter.getServoPos() <= Constants.sorterOutTakeC + 0.005){
                        sorter.setOutB();
                    }else if(sorter.getServoPos() >= Constants.sorterOutTakeB - 0.005 && sorter.getServoPos() <= Constants.sorterOutTakeB + 0.005){
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


//        } else if(gamepad2.y && gamepad2.right_trigger > 0.5) {
//            kick.kick();
//  }else{
//            kick.retract();
//        }
        }

        unsortedKick();
//
//        if(gamepad2.right_trigger > 0.5){
//            kick.kick();
//        }else{
//            kick.retract();
//        }

        if(gamepad2.left_trigger >= 0.7) {
            intake.IntakeMotorMax();
//            intake();
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
            if(sorter.getServoPos() == Constants.sorterOutTakeA){
                sorter.setOutC();
                return;
            }
            if(sorter.getServoPos() == Constants.sorterOutTakeC){
                sorter.setOutB();
                return;
            }if(sorter.getServoPos() == Constants.sorterOutTakeB){
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
            //telemetry.addLine("Sorter OB");
            sorter.setOutB();
            //telemetry.update();
        }
        if(gamepad2.x) {
            //telemetry.addLine("Sorter OC");
            sorter.setOutC();
            //telemetry.update();
        }
//        if(gamepad2.left_bumper){
//            outtakeColor(2);
//        }else if(gamepad2.right_bumper){
//            outtakeColor(1);
//        }
    }


    //    public void intake() {
//        intake.IntakeMotorMax();
//
//        servoValue = sorter.GetServoPos();
//        telemetry.addData("Servo Value ", servoValue);
//
//        if (Math.abs(servoValue - 0.03) < 0.005) {
//            sortingValues[0][1] = Constants.sorterOutTakeA;
//
//        } else if (Math.abs(servoValue - 0.105) < 0.005) {
//            sortingValues[1][1] = Constants.sorterOutTakeB;
//
//        } else if (Math.abs(servoValue - 0.17) < 0.005) {
//            sortingValues[2][1] = Constants.sorterOutTakeC;
//        }
//    }
//
//
//    public void Intake(){
//        intake.IntakeMotorMax();
//        SortingWithColor.DetectedColor detectedColor = colorSensor.getDetectedColor(telemetry);
//        servoValue = sorter.GetServoPos();
//        telemetry.addData("Detected Color ", detectedColor);
//        telemetry.addData("Servo Value ", servoValue);
//        if (Math.abs(servoValue - 0.03) < 0.005) {
//            sortingValues[0][0] = detectedColor.getCode();
//            sortingValues[0][1] = Constants.sorterOutTakeA;
//
//        } else if (Math.abs(servoValue - 0.105) < 0.005) {
//            sortingValues[1][0] = detectedColor.getCode();
//            sortingValues[1][1] = Constants.sorterOutTakeB;
//
//        } else if (Math.abs(servoValue - 0.17) < 0.005) {
//            sortingValues[2][0] = detectedColor.getCode();
//            sortingValues[2][1] = Constants.sorterOutTakeC;
//        }
//    }
//
//
//    public void outtakeAll() {
//        for (int index = 0; index < 3; index++) {
//
//            double outPos = sortingValues[index][1];
//
//            // If this slot has NOT been emptied
//            if (outPos != 0) {
//
//                // Move to the position
//                sorter.setServo(outPos);
//                sleep(300);
//
//                // Kick
//                kick.kick();
//                sleep(300);
//                kick.retract();
//
//                telemetry.addData("Outtaking slot", index);
//                telemetry.update();
//
//                // Clear this slot
//                sortingValues[index][0] = 0;
//                sortingValues[index][1] = 0;
//
//                // Move on to next filled slot on next call
//                return;
//            }
//        }
//    }
//
//
//    public void outtakeColor(int targetColor) {
//        for (int index = 0; index < 3; index++) {
//
//            double storedColor = sortingValues[index][0];
//            double outPos = sortingValues[index][1];
//
//            if (storedColor == targetColor) {
//
//                sorter.setServo(outPos);
//
//                sleep(300);
////                kick.kick();
////                sleep(3000);
////                kick.retract();
////                telemetry.addLine("Clearing out value");
//                telemetry.update();
//
//                sortingValues[index][0] = 0;
//                sortingValues[index][1] = 0;
//
//                return;
//            }
//        }
//    }
//
    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);
        //colorSensor.init(hardwareMap);
        intake.init(hardwareMap);
        sorter.init(hardwareMap);
        shooter.init(hardwareMap);
        kick.init(hardwareMap);
        led.init(hardwareMap);
        ll.init(hardwareMap);

        drive.imu.resetYaw();

//        sortingValues = new double[3][2];

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            setDriver();
            setOperator();

            telemetry.addData("aligned", Math.abs(ll.getTx()) < 1);

            telemetry.update();
        }
    }
}
