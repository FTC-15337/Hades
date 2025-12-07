package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.IntakeConfig;
import org.firstinspires.ftc.teamcode.Mechanisms.Kicker;
import org.firstinspires.ftc.teamcode.Mechanisms.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Mechanisms.ShootingConfig;
import org.firstinspires.ftc.teamcode.Mechanisms.StorageConfig;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

@Autonomous(name = "BLUE AUTO BACK")
public class BlueAutoBack extends OpMode {
    private StorageConfig sorter = new StorageConfig();
    private ShootingConfig shooter = new ShootingConfig();
    private Kicker kick = new Kicker();
    private IntakeConfig intake = new IntakeConfig();
    private MecanumDrivebase drive = new MecanumDrivebase();
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
    }

    PathState pathState;

    private final Pose startPose = new Pose(56.961493582263714, 9.241540256709445, Math.toRadians(90));
    private final Pose shootPose = new Pose(56.961493582263714, 15.290548424737455, Math.toRadians(110));

    private PathChain driveStartPosShootPos;

    //Making the paths
    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
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
        if (pathTimer.getElapsedTimeSeconds() > 2.5 && pathTimer.getElapsedTimeSeconds() < 3.2) {
            telemetry.addLine("Kicking");
            kick.kick();
        }
        if (pathTimer.getElapsedTimeSeconds() > 3.2 && pathTimer.getElapsedTimeSeconds() < 3.9) {
            kick.retract();
        }
        if(pathTimer.getElapsedTimeSeconds() > 3.9 && pathTimer.getElapsedTimeSeconds() < 4.2){
            sorter.setOutC();
        }
        if (pathTimer.getElapsedTimeSeconds() > 4.2 && pathTimer.getElapsedTimeSeconds() < 4.9) {
            kick.kick();
        }
        if (pathTimer.getElapsedTimeSeconds() > 4.9 && pathTimer.getElapsedTimeSeconds() < 5.6) {
            kick.retract();
        }
        if(pathTimer.getElapsedTimeSeconds() > 5.6 && pathTimer.getElapsedTimeSeconds() < 5.9){
            sorter.setOutB();
        }
        if (pathTimer.getElapsedTimeSeconds() > 5.9 && pathTimer.getElapsedTimeSeconds() < 6.6) {
            kick.kick();
        }
        if (pathTimer.getElapsedTimeSeconds() > 6.6 && pathTimer.getElapsedTimeSeconds() < 7.3) {
            kick.retract();
        }
        if (pathTimer.getElapsedTimeSeconds() > 7.3) {
            sorter.setIntakeA();
        }
    }
    public void firingOne() {
        telemetry.addData("Time", pathTimer.getElapsedTimeSeconds());
        if (pathTimer.getElapsedTimeSeconds() > 1 && pathTimer.getElapsedTimeSeconds() < 1.7) {
            telemetry.addLine("Kicking");
            kick.kick();
        }
        if (pathTimer.getElapsedTimeSeconds() > 1.7 && pathTimer.getElapsedTimeSeconds() < 2.4) {
            kick.retract();
        }
        if(pathTimer.getElapsedTimeSeconds() > 2.4 && pathTimer.getElapsedTimeSeconds() < 2.7){
            sorter.setOutC();
        }
        if (pathTimer.getElapsedTimeSeconds() > 2.7 && pathTimer.getElapsedTimeSeconds() < 3.4) {
            kick.kick();
        }
        if (pathTimer.getElapsedTimeSeconds() > 3.4 && pathTimer.getElapsedTimeSeconds() < 4.1) {
            kick.retract();
        }
        if(pathTimer.getElapsedTimeSeconds() > 4.1 && pathTimer.getElapsedTimeSeconds() < 4.4){
            sorter.setOutB();
        }
        if (pathTimer.getElapsedTimeSeconds() > 4.4 && pathTimer.getElapsedTimeSeconds() < 5.1) {
            kick.kick();
        }
        if (pathTimer.getElapsedTimeSeconds() > 5.1 && pathTimer.getElapsedTimeSeconds() < 5.8) {
            kick.retract();
        }
        if (pathTimer.getElapsedTimeSeconds() > 5.8) {
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
        shooter.hoodFar();
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