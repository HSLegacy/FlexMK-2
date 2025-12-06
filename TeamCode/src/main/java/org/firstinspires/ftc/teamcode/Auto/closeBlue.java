package org.firstinspires.ftc.teamcode.Auto;
import org.firstinspires.ftc.teamcode.subSystems.Spindexer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;
import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;


import static java.lang.Math.abs;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import java.util.Timer;

@Autonomous(name = "closeBlue")

public class closeBlue extends NextFTCOpMode {
    CRServoEx intake = new CRServoEx("intake");
    CRServoEx lUptake = new CRServoEx("lUptake");
    CRServoEx rUptake = new CRServoEx("rUptake");

    LLResultTypes.FiducialResult lastResult = null;
    private int lastIndex = 0;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(22, 123, Math.toRadians(90));
    private final Pose cameraPose = new Pose(60, 95, Math.toRadians(90));
    private final Pose launchPose = new Pose(60, 85, Math.toRadians(132));
    private final Pose pickUp1 = new Pose(37.5, 85.5, Math.toRadians(180));
    private final Pose pickUp2 = new Pose(29.5, 85.5, Math.toRadians(180));
    private final Pose pickUp3 = new Pose(15, 87, Math.toRadians(180));
    private final Pose launchPose2 = new Pose(60, 85, Math.toRadians(145));
    private final Pose parkPose = new Pose(55,130, Math.toRadians(180));

    public PathChain cameraPath, launchPath, pickPath1, pickPath2, pickPath3, launchPath2, parkPath;

    public void buildPaths() {
        cameraPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, cameraPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), cameraPose.getHeading())
                .build();
        launchPath = follower().pathBuilder()
                .addPath(new BezierLine(cameraPose, launchPose))
                .setLinearHeadingInterpolation(cameraPose.getHeading(), launchPose.getHeading())
                .build();
        pickPath1 = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, pickUp1))
                .setBrakingStrength(.2)
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickUp1.getHeading())
                .build();
        pickPath2 = follower().pathBuilder()
                .addPath(new BezierLine(pickUp1, pickUp2))
                .setLinearHeadingInterpolation(pickUp1.getHeading(), pickUp2.getHeading())
                .setBrakingStrength(.2)
                .build();
        pickPath3 = follower().pathBuilder()
                .addPath(new BezierLine(pickUp2, pickUp3))
                .setLinearHeadingInterpolation(pickUp2.getHeading(), pickUp3.getHeading())
                .setBrakingStrength(.5)
                .build();
        launchPath2 = follower().pathBuilder()
                .addPath(new BezierLine(pickUp3, launchPose2))
                .setLinearHeadingInterpolation(pickUp3.getHeading(), launchPose2.getHeading())
                .setBrakingStrength(1)
                .build();
        parkPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, parkPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public Command runIntake = new SetPower(intake, -1);
    public ParallelGroup runUptake = new ParallelGroup(new SetPower(lUptake, -1), new SetPower(rUptake, 1));
    public ParallelGroup stopUptake = new ParallelGroup(new SetPower(lUptake, 0), new SetPower(rUptake, 0));
    public Command shoot1 = new LambdaCommand()
            .setStart(() -> {
                telemetry.addData("started", "it defeninitlky did the thing");
                if (lastIndex == 21){
                    telemetry.addData("21", "21 workeeded");
                    Spindexer.INSTANCE.outakePos2.schedule();
                }
                else if (lastIndex == 22){
                    Spindexer.INSTANCE.outakePos1.schedule();
                }
                else if (lastIndex == 23){
                    Spindexer.INSTANCE.outakePos1.schedule();
                }
                else {
                    Spindexer.INSTANCE.outakePos2.schedule();
                }
            });
    public Command shoot2 = new LambdaCommand()
            .setStart(() -> {
                if (lastIndex == 21){
                    Spindexer.INSTANCE.outakePos1.schedule();
                }
                else if (lastIndex == 22){
                    Spindexer.INSTANCE.outakePos2.schedule();
                }
                else if (lastIndex == 23){
                    Spindexer.INSTANCE.outakePos3.schedule();
                }
                else {
                    Spindexer.INSTANCE.outakePos1.schedule();
                }
            });
    public Command shoot3 = new LambdaCommand()
            .setStart(() -> {
                if (lastIndex == 21){
                    Spindexer.INSTANCE.autoOutakePos4.schedule();
                }
                else if(lastIndex == 22){
                    Spindexer.INSTANCE.outakePos3.schedule();
                }
                else if (lastIndex == 23){
                    Spindexer.INSTANCE.outakePos2.schedule();
                }
                else {
                    Spindexer.INSTANCE.autoOutakePos4.schedule();
                }
            });
    public Command pos1 = new LambdaCommand()
            .setStart(() -> {
                Spindexer.INSTANCE.autoIntakePos4.schedule();
            });
    public Command pos2 = new LambdaCommand()
            .setStart(() -> {
                Spindexer.INSTANCE.intakePos2.schedule();
            });
    public Command pos3 = new LambdaCommand()
            .setStart(() -> {
                Spindexer.INSTANCE.intakePos3.schedule();
            });

    public Command findIndex = new LambdaCommand()
            .setStart(() -> {
                if (Turret.INSTANCE.getIndex(limelight) != 0 && Turret.INSTANCE.getIndex(limelight) != 20 && Turret.INSTANCE.getIndex(limelight) != 24 && lastIndex == 0 && Turret.INSTANCE.getIndex(limelight) != 24) {
                    lastIndex = Turret.INSTANCE.getIndex(limelight);
                    telemetry.addData("index", lastIndex);
                }
                telemetry.update();
            });
    public closeBlue() {
        addComponents(
                new SubsystemComponent(Spindexer.INSTANCE),
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
        Spindexer.INSTANCE.spindexer.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FlyWheel.INSTANCE.off.schedule();
        Spindexer.INSTANCE.intakePos1.schedule();
        buildPaths();
        follower().setStartingPose(startPose);
    }

    private Command autonomousRoutine() {
        return new ParallelGroup(
                FlyWheel.INSTANCE.on,
                new SequentialGroup(
                        new FollowPath(cameraPath),
                        findIndex,
                        new FollowPath(launchPath),
                        runIntake,
                        shoot1,
                        new Delay(.3),
                        runUptake,
                        new Delay(2),
                        shoot2,
                        new Delay(.3),
                        runUptake,
                        new Delay(2),
                        shoot3,
                        new Delay(.3),
                        runUptake,
                        new Delay(2),
                        stopUptake,
                        pos3,
                        new FollowPath(pickPath1, false, .5),
                        new Delay(.4),
                        pos2,
                        new FollowPath(pickPath2, false, .3),
                        new Delay(.4),
                        pos1,
                        new FollowPath(pickPath3, false, .5),
                        new Delay(.6),
                        new FollowPath(launchPath2),
                        shoot1,
                        new Delay(.3),
                        runUptake,
                        new Delay(2),
                        shoot2,
                        new Delay(.3),
                        runUptake,
                        new Delay(2),
                        shoot3,
                        new Delay(.3),
                        runUptake,
                        new Delay(2),
                        stopUptake,
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
