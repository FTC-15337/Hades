package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.IntakeConfig;
import org.firstinspires.ftc.teamcode.Mechanisms.MecDrivebase;
import org.firstinspires.ftc.teamcode.Mechanisms.ServoKick;
import org.firstinspires.ftc.teamcode.Mechanisms.ShooterConfig;
import org.firstinspires.ftc.teamcode.Mechanisms.StorageConfig;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

@Autonomous(name = "RED AUTO BACK")
public class RedAutoBack extends OpMode {
    private StorageConfig sorter = new StorageConfig();
    private ShooterConfig shooter = new ShooterConfig();
    private ServoKick kick = new ServoKick();
    private IntakeConfig intake = new IntakeConfig();
    private MecDrivebase drive = new MecDrivebase();
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        INTAKE_ONE,
        DRIVE_PICKUP_0NE_SHOOT_POS,
        SHOOT_ONE,
        INTAKE_TWO,
        DRIVE_PICKUP_TWO_SHOOT_POS,
        SHOOT_TWO,
        STOP
    }

    PathState pathState;

    private final Pose startPose = new Pose(87.0385064177, 9.241540256709445, Math.toRadians(90));
    private final Pose shootPose = new Pose(87.0385064177, 15.290548424737455, Math.toRadians(67.5));
    private final Pose beginPickup1 = new Pose(102.329054842, 36.789964994165686, Math.toRadians(0));
    private final Pose finalPickup1 = new Pose(132.5, 35.789964994165686, Math.toRadians(0));
    private final Pose beginPickup2 = new Pose(87.0385064177, 59.649941656942815, Math.toRadians(0));
    private final Pose finalPickup2 = new Pose(132.5, 58.649941656942815, Math.toRadians(0));
    private final Pose leave = new Pose(87.0385064177, 35, Math.toRadians(0));

    private PathChain driveStartPosShootPos, driveShootPosBPickupPos, driveBPickupFPickup, driveFPickupShootPose, driveShootPosBPickupPos2, driveBPickupFPickup2, driveFPickupShootPose2, driveShootPosLeave;

    //Making the paths
    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosBPickupPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, beginPickup1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), beginPickup1.getHeading())
                .build();

        driveBPickupFPickup = follower.pathBuilder()
                .addPath(new BezierLine(beginPickup1, finalPickup1))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        driveFPickupShootPose = follower.pathBuilder()
                .addPath(new BezierLine(finalPickup1, shootPose))
                .setLinearHeadingInterpolation(finalPickup1.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosBPickupPos2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, beginPickup2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), beginPickup2.getHeading())
                .build();

        driveBPickupFPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(beginPickup2, finalPickup2))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        driveFPickupShootPose2 = follower.pathBuilder()
                .addPath(new BezierLine(finalPickup2, shootPose))
                .setLinearHeadingInterpolation(finalPickup2.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosLeave = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leave))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leave.getHeading())
                .build();
    }

    // State Machine
    public void statePathUpdate() {
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                if(!follower.isBusy()) {
                    follower.followPath(driveStartPosShootPos, true);
                    telemetry.addLine("Going to shoot");
                    setPathState(PathState.SHOOT_PRELOAD); // reset timer and make a new state
                }
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    firingPre();
                    telemetry.addLine("Shooting");
                    if(pathTimer.getElapsedTimeSeconds() > 8.5){
                        follower.followPath(driveShootPosBPickupPos);
                        telemetry.addLine("Going to Intake");
                        setPathState(PathState.INTAKE_ONE);
                    }
                }
                break;
            case INTAKE_ONE:
                if(!follower.isBusy()) {
                    intake.IntakeMotorMax();
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.1){
                        follower.setMaxPower(0.3);
                        telemetry.addLine("Intaking");
                        follower.followPath(driveBPickupFPickup, true);
                        telemetry.addLine("Going to shoot");
                        setPathState(PathState.DRIVE_PICKUP_0NE_SHOOT_POS);
                    }
                }
                break;
            case DRIVE_PICKUP_0NE_SHOOT_POS:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    sorter.setOutA();
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.1) {
                        follower.setMaxPower(1.0);
                        telemetry.addLine("Going to Shoot");
                        follower.followPath(driveFPickupShootPose, true);
                        setPathState(PathState.SHOOT_ONE);
                    }
                }
                break;
            case SHOOT_ONE:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){ // not using sleep as it stops the whole thread so using timer instead
                    firingOne();
                    telemetry.addLine("Shooting");
                    if(pathTimer.getElapsedTimeSeconds() > 7) {
                        // transition to next state
                        follower.followPath(driveShootPosBPickupPos2, true);
                        telemetry.addLine("Going to intake");
                        setPathState(PathState.INTAKE_TWO);
                    }
                }
                break;
            case INTAKE_TWO:
                if(!follower.isBusy()) {
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.25) {
                        follower.setMaxPower(0.3);
                        telemetry.addLine("Intaking");
                        follower.followPath(driveBPickupFPickup2, true);
                        setPathState(PathState.DRIVE_PICKUP_TWO_SHOOT_POS);
                    }
                }
                break;
            case DRIVE_PICKUP_TWO_SHOOT_POS:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    sorter.setOutA();
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.1) {
                        follower.setMaxPower(1.0);
                        telemetry.addLine("Going to shoot");
                        follower.followPath(driveFPickupShootPose2, true);
                        setPathState(PathState.SHOOT_TWO);
                    }
                }
                break;
            case SHOOT_TWO:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){ // not using sleep as it stops the whole thread so using timer instead
                    firingTwo();
                    telemetry.addLine("Shooting");
                    if(pathTimer.getElapsedTimeSeconds() > 6.5) {
                        // transition to next state
                        follower.followPath(driveShootPosLeave, true);
                        telemetry.addLine("Leaving");
                        setPathState(PathState.STOP);
                    }
                }
                break;
            case STOP:
                if(!follower.isBusy()){
                    drive.frontLeft.setPower(0.0);
                    drive.frontRight.setPower(0.0);
                    drive.frontLeft.setPower(0.0);
                    drive.frontLeft.setPower(0.0);
                }
            default:
                telemetry.addLine("Not in any state");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void firingPre() {
        telemetry.addData("Time", pathTimer.getElapsedTimeSeconds());
        if (pathTimer.getElapsedTimeSeconds() > 3.5 && pathTimer.getElapsedTimeSeconds() < 4.2) {
            telemetry.addLine("Kicking");
            kick.kick();
        }
        if (pathTimer.getElapsedTimeSeconds() > 4.2 && pathTimer.getElapsedTimeSeconds() < 4.9) {
            kick.retract();
        }
        if(pathTimer.getElapsedTimeSeconds() > 4.9 && pathTimer.getElapsedTimeSeconds() < 5.3){
            sorter.setOutC();
        }
        if (pathTimer.getElapsedTimeSeconds() > 5.3 && pathTimer.getElapsedTimeSeconds() < 6.0) {
            kick.kick();
        }
        if (pathTimer.getElapsedTimeSeconds() > 6.0 && pathTimer.getElapsedTimeSeconds() < 6.7) {
            kick.retract();
        }
        if(pathTimer.getElapsedTimeSeconds() > 6.7 && pathTimer.getElapsedTimeSeconds() < 7.1){
            sorter.setOutB();
        }
        if (pathTimer.getElapsedTimeSeconds() > 7.1 && pathTimer.getElapsedTimeSeconds() < 7.8) {
            kick.kick();
        }
        if (pathTimer.getElapsedTimeSeconds() > 7.8 && pathTimer.getElapsedTimeSeconds() < 8.5) {
            kick.retract();
        }
        if (pathTimer.getElapsedTimeSeconds() > 8.5) {
            sorter.setIntakeA();
        }
    }
    public void firingOne() {
        telemetry.addData("Time", pathTimer.getElapsedTimeSeconds());
        if (pathTimer.getElapsedTimeSeconds() > 1.8 && pathTimer.getElapsedTimeSeconds() < 2.5) {
            telemetry.addLine("Kicking");
            kick.kick();
        }
        if (pathTimer.getElapsedTimeSeconds() > 2.5 && pathTimer.getElapsedTimeSeconds() < 3.2) {
            kick.retract();
        }
        if(pathTimer.getElapsedTimeSeconds() > 3.2 && pathTimer.getElapsedTimeSeconds() < 3.6){
            sorter.setOutC();
        }
        if (pathTimer.getElapsedTimeSeconds() > 3.6 && pathTimer.getElapsedTimeSeconds() < 4.3) {
            kick.kick();
        }
        if (pathTimer.getElapsedTimeSeconds() > 4.3 && pathTimer.getElapsedTimeSeconds() < 5.0) {
            kick.retract();
        }
        if(pathTimer.getElapsedTimeSeconds() > 5.0 && pathTimer.getElapsedTimeSeconds() < 5.4){
            sorter.setOutB();
        }
        if (pathTimer.getElapsedTimeSeconds() > 5.4 && pathTimer.getElapsedTimeSeconds() < 6.1) {
            kick.kick();
        }
        if (pathTimer.getElapsedTimeSeconds() > 6.1 && pathTimer.getElapsedTimeSeconds() < 6.8) {
            kick.retract();
        }
        if (pathTimer.getElapsedTimeSeconds() > 6.8) {
            sorter.setIntakeA();
        }
    }

    public void firingTwo() {
        telemetry.addData("Time", pathTimer.getElapsedTimeSeconds());
        if (pathTimer.getElapsedTimeSeconds() > 2 && pathTimer.getElapsedTimeSeconds() < 2.7) {
            telemetry.addLine("Kicking");
            kick.kick();
        }
        if (pathTimer.getElapsedTimeSeconds() > 2.7 && pathTimer.getElapsedTimeSeconds() < 3.4) {
            kick.retract();
        }
        if(pathTimer.getElapsedTimeSeconds() > 3.4 && pathTimer.getElapsedTimeSeconds() < 3.7){
            sorter.setOutC();
        }
        if (pathTimer.getElapsedTimeSeconds() > 3.7 && pathTimer.getElapsedTimeSeconds() < 4.4) {
            kick.kick();
        }
        if (pathTimer.getElapsedTimeSeconds() > 4.4 && pathTimer.getElapsedTimeSeconds() < 5.1) {
            kick.retract();
            telemetry.addLine("After 2nd kick");
        }
        if(pathTimer.getElapsedTimeSeconds() > 5.1 && pathTimer.getElapsedTimeSeconds() < 5.4){
            sorter.setOutB();
            telemetry.addLine("In third kick");
        }
        if (pathTimer.getElapsedTimeSeconds() > 5.4 && pathTimer.getElapsedTimeSeconds() < 6.1) {
            kick.kick();
        }
        if (pathTimer.getElapsedTimeSeconds() > 6.1) {
            kick.retract();
        }
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        shooter.init(hardwareMap);
        sorter.init(hardwareMap);
        kick.init(hardwareMap);
        intake.init(hardwareMap);
        drive.init(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        shooter.hoodAutoFar();
        kick.retract();
        shooter.FarOut();
        sorter.setOutA();
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
    }
}