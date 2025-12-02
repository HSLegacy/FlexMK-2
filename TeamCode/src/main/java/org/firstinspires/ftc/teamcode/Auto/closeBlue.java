package org.firstinspires.ftc.teamcode.Auto;
import org.firstinspires.ftc.teamcode.subSystems.Turret;
import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;


import static java.lang.Math.abs;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.extensions.pedro.TurnTo;
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

@Autonomous(name = "closeBlue")

public class closeBlue extends NextFTCOpMode {
    CRServoEx intake = new CRServoEx("intake");
    CRServoEx lUptake = new CRServoEx("lUptake");
    CRServoEx rUptake = new CRServoEx("rUptake");

    LLResultTypes.FiducialResult lastResult = null;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(22, 123, Math.toRadians(90));
    private final Pose cameraPose = new Pose(60, 95, Math.toRadians(90));
    private final Pose launchPose = new Pose(60, 85, Math.toRadians(128));
    private final Pose parkPose = new Pose(55,130, Math.toRadians(180));

    public PathChain cameraPath, launchPath, parkPath;

    public void buildPaths() {
        cameraPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, cameraPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), cameraPose.getHeading())
                .build();
        launchPath = follower().pathBuilder()
                .addPath(new BezierLine(cameraPose, launchPose))
                .setLinearHeadingInterpolation(cameraPose.getHeading(), launchPose.getHeading())
                .build();
        parkPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, parkPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public Command runIntake = new SetPower(intake, -1);



// Example
    public ParallelGroup runUptake = new ParallelGroup(new SetPower(lUptake, -1), new SetPower(rUptake, 1));

    public closeBlue() {
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
                        new FollowPath(cameraPath),
                        new Delay(2),
                        new FollowPath(launchPath),
                        new Delay(1),
                        runUptake,
                        new Delay(5),
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
