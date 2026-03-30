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

    Turret turret = Turret.getInstance(limelight, telemetry);

    private final Pose startPose = new Pose(15, 122, Math.toRadians(180));
    private final Pose launchPose = new Pose(55, 90, Math.toRadians(180));
    private final Pose launchPose2 = new Pose(55, 109, Math.toRadians(180));
    private final Pose spike2Spot1 = new Pose(44, 64, Math.toRadians(180));
    private final Pose spike2Spot2 = new Pose(11, 64, Math.toRadians(180));
    private final Pose spike1Pose = new Pose(16, 80, Math.toRadians(180));
    private final Pose midpoint = new Pose(30, 55, Math.toRadians(180));
    private final Pose takeFromGatePose = new Pose(15,71, Math.toRadians(180));
    private final Pose pickUp = new Pose(10, 61, Math.toRadians(148));
    private final Pose midpoint2 = new Pose(19, 71, Math.toRadians(160));


    public PathChain launchPath, spike2, launchPath2, takeFromGatePath, pickUpPath, launchPath3, takeFromGatePath2, pickUpPath2, launchPath4, spike1Path, launchPath5;

    public void buildPaths() {
        launchPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .addParametricCallback(.95, openGate)
                .build();
        spike2 = follower().pathBuilder()
                .addPath(new BezierCurve(launchPose, spike2Spot1, spike2Spot2))
                .setLinearHeadingInterpolation(launchPose.getHeading(), spike2Spot2.getHeading())
                .build();
        launchPath2 = follower().pathBuilder()
                .addPath(new BezierLine(spike2Spot2, launchPose))
                .setLinearHeadingInterpolation(spike2Spot2.getHeading(), launchPose.getHeading())
                .addParametricCallback(.95, openGate)
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
                .addPath(new BezierLine(pickUp, launchPose))
                .setLinearHeadingInterpolation(pickUp.getHeading(), launchPose.getHeading())
                .addParametricCallback(.90, openGate)
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
                .addPath(new BezierLine(pickUp, launchPose))
                .setLinearHeadingInterpolation(pickUp.getHeading(), launchPose.getHeading())
                .addParametricCallback(.90, openGate)
                .build();
        spike1Path = follower().pathBuilder()
                .addPath(new BezierCurve(launchPose, spike1Pose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), spike1Pose.getHeading())
                .build();
        launchPath5 = follower().pathBuilder()
                .addPath(new BezierLine(spike1Pose, launchPose2))
                .setLinearHeadingInterpolation(spike1Pose.getHeading(), launchPose2.getHeading())
                .addParametricCallback(.90, openGate)
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
                turret.resetButton();
            });
    public closeBlue() {
        addComponents(
                new SubsystemComponent(turret),
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
        turret.turretMotor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                closeGate,
                runIntake,
                new FollowPath(launchPath), //also opens gate
                relocalize,
                new Delay(2),
                closeGate,
                new FollowPath(spike2),
                new FollowPath(launchPath2), //also opens gate
                relocalize,
                new Delay(2),
                closeGate,
                new FollowPath(takeFromGatePath),
                new FollowPath(pickUpPath),
                new Delay(2),
                new FollowPath(launchPath3), //also opens gate
                relocalize,
                new Delay(2),
                closeGate,
                new FollowPath(takeFromGatePath2),
                new FollowPath(pickUpPath2),
                new Delay(2),
                new FollowPath(launchPath4),
                relocalize,
                new Delay(2),
                closeGate,
                new FollowPath(spike1Path),
                new FollowPath(launchPath5)
        );
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
        FlyWheel.INSTANCE.isStarted = true;
        turret.opModeIsStarted = true;
    }


    @Override
    public void onUpdate() {
        FlyWheel.INSTANCE.setGoal(turret.flyWheelGoal);

        turret.relocalizationUpdate(limelight, telemetry);
        turret.autoFlyWheelRegressionBlue(limelight, telemetry);
        turret.turretMovement(false);

        telemetry.addData("Flywheel Goal", turret.flyWheelGoal);
        telemetry.update();
    }

    public static Limelight3A limelight = null;

    @Override
    public void onStop() {

    }


}