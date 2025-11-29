package org.firstinspires.ftc.teamcode.opmodes; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BLUE AUTO TOP")
public class BlueAutoTop extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(21.84, 122.16, Math.toRadians(130)); // Start Pose of our robot.
    private final Pose firing = new Pose(47.56, 95.78, Math.toRadians(130)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose startPickup1 = new Pose(47.72, 84.18, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose endPickup1 = new Pose(14.79, 83.85, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose startPickup2 = new Pose(47.38, 59.65, Math.toRadians(180));
    private final Pose endPickup2 = new Pose(15.29, 59.65, Math.toRadians(180));
    private final Pose leave = new Pose(61.50, 36.29, Math.toRadians(180));
    private PathChain scorePreload, beginPickup1, finalPickup1, scorePickup1, beginPickup2, finalPickup2, scorePickup2, beginPickup3, finalPickup3, scorePickup3, leaving;

    public void buildPaths() {
        scorePreload = follower
                .pathBuilder()
                .addPath(new BezierLine(startPose, firing))
                .setConstantHeadingInterpolation(Math.toRadians(130))
                .build();

        beginPickup1 = follower
                .pathBuilder()
                .addPath(new BezierLine(firing, startPickup1))
                .setLinearHeadingInterpolation(firing.getHeading(), startPickup1.getHeading())
                .build();

        finalPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(startPickup1, endPickup1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(endPickup1, firing))
                .setLinearHeadingInterpolation(endPickup1.getHeading(), firing.getHeading())
                .build();

        beginPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(firing, startPickup2))
                .setLinearHeadingInterpolation(firing.getHeading(), startPickup2.getHeading())
                .build();

        finalPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(startPickup2, endPickup2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(endPickup2, firing))
                .setLinearHeadingInterpolation(endPickup2.getHeading(), firing.getHeading())
                .build();

        leaving = follower.pathBuilder()
                .addPath(new BezierLine(firing, leave))
                .setLinearHeadingInterpolation(firing.getHeading(), leave.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(0.75);
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.75);
                    follower.followPath(beginPickup1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.25);
                    follower.followPath(finalPickup1,true);
                    setPathState(3);

                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.75);
                    follower.followPath(scorePickup1,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.75);
                    follower.followPath(beginPickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.25);
                    follower.followPath(finalPickup2,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.75);
                    follower.followPath(scorePickup2, true);
                    setPathState(7);
                }

            case 7:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.75);
                    follower.followPath(leaving, true);
                    setPathState(8);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}
