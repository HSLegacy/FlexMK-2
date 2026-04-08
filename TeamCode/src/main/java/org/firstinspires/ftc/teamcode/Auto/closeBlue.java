package org.firstinspires.ftc.teamcode.Auto;
import org.firstinspires.ftc.teamcode.subSystems.Turret;
import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;


import static java.lang.Math.abs;

import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import java.util.Timer;

@Autonomous(name = "closeBlue")

public class closeBlue extends NextFTCOpMode {
    MotorEx intake = new MotorEx("intake");
    CRServoEx uptake = new CRServoEx("uptake");
    ServoEx gate = new ServoEx("door");

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;


    private final Pose startPose = new Pose(20, 122, Math.toRadians(180));
    private final Pose launchPose = new Pose(53, 92, Math.toRadians(180));
    private final Pose launchPose2 = new Pose(55, 113, Math.toRadians(180));
    private final Pose launchPose3 = new Pose(25, 110, Math.toRadians(180));
    private final Pose goToSpike2 = new Pose(62, 65, Math.toRadians(180));
    private final Pose spike2Spot1 = new Pose(56, 60, Math.toRadians(180));
    private final Pose spike2Spot2 = new Pose(5, 58, Math.toRadians(180));
    private final Pose spike1Pose = new Pose(17.5, 86, Math.toRadians(180));
    private final Pose midpoint = new Pose(47.5, 49, Math.toRadians(180));
    private final Pose takeFromGatePose = new Pose(16.5,68, Math.toRadians(180));
    private final Pose pickUp = new Pose(9, 60, Math.toRadians(142));
    private final Pose midpoint2 = new Pose(21, 67, Math.toRadians(160));


    public PathChain launchPath, spike2, launchPath2, takeFromGatePath, pickUpPath, launchPath3, takeFromGatePath2, pickUpPath2, launchPath4, spike1Path, launchPath5, launchPath6;

    public void buildPaths() {
        launchPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();
        spike2 = follower().pathBuilder()
                .addPath(new BezierCurve(startPose, goToSpike2, spike2Spot2))
                .setLinearHeadingInterpolation(startPose.getHeading(), spike2Spot2.getHeading())
                .build();
        launchPath2 = follower().pathBuilder()
                .addPath(new BezierCurve(spike2Spot2, midpoint, launchPose))
                .setLinearHeadingInterpolation(spike2Spot2.getHeading(), launchPose.getHeading())
                .build();
        takeFromGatePath = follower().pathBuilder()
                .addPath(new BezierCurve(launchPose, midpoint, takeFromGatePose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), takeFromGatePose.getHeading())
                .build();
        pickUpPath = follower().pathBuilder()
                .addPath(new BezierCurve(takeFromGatePose, midpoint2, pickUp))
                .setLinearHeadingInterpolation(takeFromGatePose.getHeading(), pickUp.getHeading())
                .build();
        launchPath3 = follower().pathBuilder()
                .addPath(new BezierCurve(pickUp, midpoint, launchPose))
                .setLinearHeadingInterpolation(pickUp.getHeading(), launchPose.getHeading())
                .build();
        takeFromGatePath2 = follower().pathBuilder()
                .addPath(new BezierCurve(launchPose, midpoint, takeFromGatePose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), takeFromGatePose.getHeading())
                .build();
        pickUpPath2 = follower().pathBuilder()
                .addPath(new BezierCurve(takeFromGatePose, midpoint2, pickUp))
                .setLinearHeadingInterpolation(takeFromGatePose.getHeading(), pickUp.getHeading())
                .build();
        launchPath4 = follower().pathBuilder()
                .addPath(new BezierCurve(pickUp, midpoint,launchPose))
                .setLinearHeadingInterpolation(pickUp.getHeading(), launchPose.getHeading())
                .build();
        spike1Path = follower().pathBuilder()
                .addPath(new BezierCurve(launchPose, spike1Pose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), spike1Pose.getHeading())
                .build();
        launchPath5 = follower().pathBuilder()
                .addPath(new BezierLine(spike1Pose, launchPose2))
                .setLinearHeadingInterpolation(spike1Pose.getHeading(), launchPose2.getHeading())
                .build();
        launchPath6 = follower().pathBuilder()
                .addPath(new BezierLine(startPose, launchPose3))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose3.getHeading())
                .build();
    }

    public Command runIntake = new LambdaCommand()
            .setStart(() -> {
                intake.setPower(-1);
                uptake.setPower(1);
            });

    public Command openGate = new LambdaCommand()
            .setStart(() -> {
                gate.setPosition(1);
            });
    public Command closeGate = new LambdaCommand()
            .setStart(() -> {
                gate.setPosition(0.5);
            });
    public Command relocalize = new LambdaCommand()
            .setStart(() -> {
                Turret.INSTANCE.resetButton();
            });
    public Command setNewTurretPose = new LambdaCommand()
            .setStart(() -> {
                Turret.INSTANCE.targetPoseBlue = new Pose(2, 142);
            });
    public Command setOldTurretPose = new LambdaCommand()
            .setStart(() -> {
                Turret.INSTANCE.targetPoseBlue = new Pose(4, 144);
            });
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
        Turret.INSTANCE.turretMotor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Turret.INSTANCE.limelight = limelight;
        Turret.INSTANCE.telemetry = telemetry;

    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                setNewTurretPose,
                closeGate,
                runIntake,
                new FollowPath(launchPath6),
                openGate,
                new Delay(2),
                closeGate,
                new FollowPath(spike2),
                new FollowPath(launchPath2), //also opens gate
                openGate,
                new Delay(2),
                closeGate,
                new FollowPath(takeFromGatePath),
                new FollowPath(pickUpPath),
                new Delay(1),
                new FollowPath(launchPath3), //also opens gate
                setOldTurretPose,
                openGate,
                new Delay(2),
                closeGate,
                new FollowPath(takeFromGatePath2),
                new FollowPath(pickUpPath2),
                new Delay(1),
                new FollowPath(launchPath4),
                openGate,
                new Delay(2),
                closeGate,
                new FollowPath(spike1Path),
                new FollowPath(launchPath5),
                openGate
        );
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
        FlyWheel.INSTANCE.isStarted = true;
        Turret.INSTANCE.opModeIsStarted = true;
    }


    @Override
    public void onUpdate() {
        FlyWheel.INSTANCE.setGoal(Turret.INSTANCE.flyWheelGoal);

        Turret.INSTANCE.relocalizationUpdate(limelight, telemetry);
        Turret.INSTANCE.autoFlyWheelRegressionBlue(telemetry);
        Turret.INSTANCE.turretMovement(false);

        telemetry.addData("Flywheel Goal", Turret.INSTANCE.flyWheelGoal);
        telemetry.update();
    }

    public static Limelight3A limelight = null;

    @Override
    public void onStop() {

    }


}