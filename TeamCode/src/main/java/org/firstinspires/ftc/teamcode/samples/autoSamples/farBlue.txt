package org.firstinspires.ftc.teamcode.samples.autoSamples;
import org.firstinspires.ftc.teamcode.subSystems.Turret;
import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;

import static java.lang.Math.abs;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import java.util.Timer;

@Autonomous(name = "farBlue")

public class farBlue extends NextFTCOpMode {

    CRServoEx intake = new CRServoEx("intake");
    ServoEx leftRelease = new ServoEx("lbar");
    ServoEx rightRelease = new ServoEx("rbar");
    ServoEx triggerServo = new ServoEx("launch");
    MotorEx rightFlyWheel = new MotorEx("rfw");
    MotorEx leftFlyWheel = new MotorEx("lfw");

    LLResultTypes.FiducialResult lastResult = null;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(52, 9, Math.toRadians(270));
    private final Pose fowardPose = new Pose(52, 75, Math.toRadians(270));
    private final Pose launchPose = new Pose(37, 95, Math.toRadians(-52));
    private final Pose launchPose2 = new Pose(34, 97, Math.toRadians(-52));
    private final Pose pickUpPose = new Pose(46, 83, Math.toRadians(180));
    private final Pose pickUp = new Pose(18, 83, Math.toRadians(180));
    private final Pose parkPose = new Pose(37,70, Math.toRadians(180));
    private final Pose pickUpPose2 = new Pose(44, 60, Math.toRadians(180));
    private final Pose pickUp2 = new Pose(12, 60, Math.toRadians(180));
    private final Pose midpoint = new Pose(35, 70, Math.toRadians(180));

    public PathChain driveForward, launchPath, parkPath, pickUpBalls, launchPath2, pickUpBalls2, launchPath3;

    public void buildPaths() {

        driveForward = follower().pathBuilder()
                .addPath(new BezierLine(startPose, fowardPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), fowardPose.getHeading())
                .build();
        launchPath = follower().pathBuilder()
                .addPath(new BezierLine(fowardPose, launchPose))
                .setLinearHeadingInterpolation(fowardPose.getHeading(), launchPose.getHeading())
                .build();
        parkPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, parkPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), parkPose.getHeading())
                .build();
        pickUpBalls = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, pickUpPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickUpPose.getHeading())
                .addPath(new BezierLine(pickUpPose, pickUp))
                .setLinearHeadingInterpolation(pickUpPose.getHeading(), pickUp.getHeading())
                .build();
        launchPath2 = follower().pathBuilder()
                .addPath(new BezierLine(pickUp, launchPose2))
                .setLinearHeadingInterpolation(pickUp.getHeading(), launchPose2.getHeading())
                .build();
        pickUpBalls2 = follower().pathBuilder()
                .addPath(new BezierLine(launchPose2, pickUpPose2))
                .setLinearHeadingInterpolation(launchPose2.getHeading(), pickUpPose2.getHeading())
                .addPath(new BezierLine(pickUpPose2, pickUp2))
                .setLinearHeadingInterpolation(pickUpPose2.getHeading(), pickUp2.getHeading())
                .build();
        launchPath3 = follower().pathBuilder()
                .addPath(new BezierLine(pickUp2, midpoint))
                .setLinearHeadingInterpolation(pickUp2.getHeading(), midpoint.getHeading())
                .addPath(new BezierLine(midpoint, launchPose2))
                .setLinearHeadingInterpolation(midpoint.getHeading(), launchPose2.getHeading())
                .build();
    }

    public Command launchBall = new SetPosition(triggerServo, .4);
    public Command theDownies = new SetPosition(triggerServo, .6);
    public Command rightGateOpen = new SetPosition(rightRelease, 0);
    public Command rightGateClose = new SetPosition(rightRelease, .4);
    public Command leftGateOpen = new SetPosition(leftRelease, .8);
    public Command leftGateClose = new SetPosition(leftRelease, .3);
    public Command runIntake = new SetPower(intake, -1);

    public farBlue() {
        addComponents(
                new SubsystemComponent(Turret.INSTANCE),
                new SubsystemComponent(FlyWheel.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
        FlyWheel.INSTANCE.off.schedule();
        buildPaths();
        follower().setStartingPose(startPose);
    }

    private Command autonomousRoutine() {
        return new ParallelGroup(
                FlyWheel.INSTANCE.autoOn,
                rightGateClose,
                leftGateClose,
                new SequentialGroup(
                        new FollowPath(driveForward),
                        new FollowPath(launchPath),
                        launchBall,
                        new Delay(.5),
                        theDownies, //Henry don't hate us please
                        rightGateOpen,
                        runIntake,
                        new Delay(2),
                        launchBall,
                        new Delay(.5),
                        theDownies, //Henry don't hate us please
                        leftGateOpen,
                        new Delay(2),
                        launchBall,
                        new Delay(.5),
                        theDownies, //Henry don't hate us please
                        //new FollowPath(parkPath)
                        new ParallelGroup(
                                rightGateClose,
                                leftGateClose
                        ),
                        new FollowPath(pickUpBalls, true, .7),
                        new Delay(1),
                        new FollowPath(launchPath2),
                        new Delay(.5),
                        launchBall,
                        new Delay(.5),
                        theDownies,
                        new Delay(2),
                        launchBall,
                        new Delay(.5),
                        theDownies,
                        new FollowPath(pickUpBalls2, true, .7),
                        new FollowPath(launchPath3),
                        new Delay(.5),
                        launchBall,
                        new Delay(.5),
                        theDownies,
                        new Delay(2),
                        launchBall,
                        new Delay(.5),
                        theDownies,
                        new FollowPath(parkPath)
                )
        );
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate() {

    }

    public static Limelight3A limelight = null;

    @Override
    public void onStop() {

    }


}
