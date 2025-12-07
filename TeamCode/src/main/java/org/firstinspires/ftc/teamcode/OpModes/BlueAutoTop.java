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
import org.firstinspires.ftc.teamcode.Mechanisms.MecDrivebase;
import org.firstinspires.ftc.teamcode.Mechanisms.ServoKick;
import org.firstinspires.ftc.teamcode.Mechanisms.ShooterConfig;
import org.firstinspires.ftc.teamcode.Mechanisms.StorageConfig;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

@Autonomous(name = "BLUE AUTO TOP")
public class BlueAutoTop extends OpMode {
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
        DRIVE_BPICKUP_FPICKUP, // BPICKUP is begin pickup and EPICKUP is end pickup
        DRIVE_PICKUP1_SHOOT_POS,
        SHOOT_ONE,
        DRIVE_BPICKUP_FPICKUP2,
        DRIVE_PICKUP2_SHOOT_POS,
        SHOOT_TWO,
        LEAVE,
        STOP
    }

    PathState pathState;

    private final Pose startPose = new Pose(21.843640606767796, 122.15635939323221, Math.toRadians(130));
    private final Pose shootPose = new Pose(47.55192532088682, 95.77596266044341, Math.toRadians(130));

    private final Pose beginPickup1 = new Pose(47.71995332555426, 84.18203033838974, Math.toRadians(180));
    private final Pose finalPickup1 = new Pose(15.786464410735121, 83.84597432905484, Math.toRadians(180));

    private final Pose beginPickup2 = new Pose(47.38389731621937, 59.649941656942815, Math.toRadians(180));
    private final Pose finalPickup2 = new Pose(4, 59.649941656942815, Math.toRadians(180));
    private final Pose midCurveFPickupShoot = new Pose(45.26829268292683, 36.8780487804878);
    private final Pose leave = new Pose(52.68292682926829, 126.82926829268293, Math.toRadians(180));

    private PathChain driveStartPosShootPos, driveShootPosBPickupPos, driveBPoseFPose, driveFPoseShootPose, driveShootPosBPickup2, driveBPoseFPose2, driveFPoseShootPose2, driveShootPosLeave;

    //Making the paths
    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setConstantHeadingInterpolation(Math.toRadians(130))
                .build();

        driveShootPosBPickupPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, beginPickup1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), beginPickup1.getHeading())
                .build();

        driveBPoseFPose = follower.pathBuilder()
                .addPath(new BezierLine(beginPickup1, finalPickup1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        driveFPoseShootPose = follower.pathBuilder()
                .addPath(new BezierLine(finalPickup1, shootPose))
                .setLinearHeadingInterpolation(finalPickup1.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosBPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, beginPickup2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), beginPickup2.getHeading())
                .build();

        driveBPoseFPose2 = follower.pathBuilder()
                .addPath(new BezierLine(beginPickup2, finalPickup2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        driveFPoseShootPose2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        finalPickup2,
                        midCurveFPickupShoot,
                        shootPose))
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
                follower.followPath(driveStartPosShootPos, true);
                telemetry.addLine("Going to shoot");
                setPathState(PathState.SHOOT_PRELOAD); // reset timer and make a new state
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){ // not using sleep as it stops the whole thread so using timer instead
                    firingPre();
                    telemetry.addLine("Shooting");
                    if(pathTimer.getElapsedTimeSeconds() > 8) {                                     //<- optimize if needed
                        // transition to next state
                        follower.followPath(driveShootPosBPickupPos, true);
                        telemetry.addLine("Going to intake");
                        setPathState(PathState.DRIVE_BPICKUP_FPICKUP);
                    }
                }
                break;
            case DRIVE_BPICKUP_FPICKUP:
                if(!follower.isBusy()) {
                    intake.IntakeMotorMax();
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.1) {
                        follower.setMaxPower(0.5);
                        telemetry.addLine("Intaking");
                        follower.followPath(driveBPoseFPose, true);
                        telemetry.addLine("Going to shoot");
                        setPathState(PathState.DRIVE_PICKUP1_SHOOT_POS);
                    }
                }
                break;
            case DRIVE_PICKUP1_SHOOT_POS:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    sorter.setOutA();
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.1) {
                        follower.setMaxPower(1.0);
                        telemetry.addLine("Going to shoot");
                        follower.followPath(driveFPoseShootPose, true);
                        setPathState(PathState.SHOOT_ONE);
                    }
                }
                break;
            case SHOOT_ONE:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.1){ // not using sleep as it stops the whole thread so using timer instead
                    firingOne();
                    telemetry.addLine("Shooting");
                    if(pathTimer.getElapsedTimeSeconds() > 6) {
                        // transition to next state
                        follower.followPath(driveShootPosBPickup2, true);
                        telemetry.addLine("Going to intake");
                        setPathState(PathState.DRIVE_BPICKUP_FPICKUP2);
                    }
                }
                break;
            case DRIVE_BPICKUP_FPICKUP2:
                if(!follower.isBusy()) {
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.25) {
                        follower.setMaxPower(0.5);
                        telemetry.addLine("Intaking");
                        follower.followPath(driveBPoseFPose2, true);
                        setPathState(PathState.DRIVE_PICKUP2_SHOOT_POS);
                    }
                }
                break;
            case DRIVE_PICKUP2_SHOOT_POS:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    sorter.setOutA();
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.1) {
                        follower.setMaxPower(1.0);
                        telemetry.addLine("Going to shoot");
                        follower.followPath(driveFPoseShootPose2, true);
                        setPathState(PathState.SHOOT_TWO);
                    }
                }
                break;
            case SHOOT_TWO:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.1){ // not using sleep as it stops the whole thread so using timer instead
                    firingTwo();
                    telemetry.addLine("Shooting");
                    if(pathTimer.getElapsedTimeSeconds() > 6) {
                        setPathState(PathState.LEAVE);
                    }
                }
                break;
            case LEAVE:
                if(!follower.isBusy()){
                    follower.followPath(driveShootPosLeave, true);
                    telemetry.addLine("Leaving");
                    setPathState(PathState.STOP);
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
        shooter.hoodClose();
        kick.retract();
        shooter.CloseOut();
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