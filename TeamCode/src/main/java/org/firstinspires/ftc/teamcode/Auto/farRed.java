package org.firstinspires.ftc.teamcode.Auto;
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
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import java.util.Timer;

@Autonomous(name = "farRed")

public class farRed extends NextFTCOpMode {

    CRServoEx intake = new CRServoEx("intake");
    CRServoEx lUptake = new CRServoEx("lUptake");
    CRServoEx rUptake = new CRServoEx("rUptake");

    LLResultTypes.FiducialResult lastResult = null;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(52, 9, Math.toRadians(90))
            .mirror();
    private final Pose fowardPose = new Pose(52, 75, Math.toRadians(90))
            .mirror();
    private final Pose launchPose = new Pose(54, 85, Math.toRadians(128))
            .mirror();
    private final Pose parkPose = new Pose(45,70, Math.toRadians(180))
            .mirror();

    public PathChain driveForward, launchPath, parkPath;

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
    }

    public Command runIntake = new SetPower(intake, -1);
    public ParallelGroup runUptake = new ParallelGroup(new SetPower(lUptake, -1), new SetPower(rUptake, 1));

    public farRed() {
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
                FlyWheel.INSTANCE.on,
                new SequentialGroup(
                        new FollowPath(driveForward),
                        new FollowPath(launchPath),
                        new Delay(2),
                        runUptake,
                        new Delay(2),
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
