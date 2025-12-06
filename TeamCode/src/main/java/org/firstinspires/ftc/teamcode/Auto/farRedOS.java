package org.firstinspires.ftc.teamcode.Auto;
import org.firstinspires.ftc.teamcode.subSystems.Spindexer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;
import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;

import static java.lang.Math.abs;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import java.util.Timer;

@Autonomous(name = "farRedOS")

public class farRedOS extends NextFTCOpMode {

    CRServoEx intake = new CRServoEx("intake");
    CRServoEx lUptake = new CRServoEx("lUptake");
    CRServoEx rUptake = new CRServoEx("rUptake");

    LLResultTypes.FiducialResult lastResult = null;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private int lastIndex = 0;
    private final Pose startPose = new Pose(52, 9, Math.toRadians(90))
            .mirror();
    private final Pose launchPose = new Pose(52, 15, Math.toRadians(113))
            .mirror();
    private final Pose parkPose = new Pose(38, 15, Math.toRadians(90))
            .mirror();

    public PathChain launchPath, parkPath;

    public void buildPaths() {

        launchPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();
        parkPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, parkPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), parkPose.getHeading())
                .build();
    }
    public ParallelGroup runUptake = new ParallelGroup(new SetPower(lUptake, -1), new SetPower(rUptake, 1));
    public ParallelGroup stopUptake = new ParallelGroup(new SetPower(lUptake, 0), new SetPower(rUptake, 0));
    public Command runIntake = new SetPower(intake, -1);
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


    public farRedOS() {
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
        FlyWheel.INSTANCE.off.schedule();
        buildPaths();
        Spindexer.INSTANCE.intakePos1.schedule();
        Spindexer.INSTANCE.spindexer.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        follower().setStartingPose(startPose);
    }

    private Command autonomousRoutine() {
        return new ParallelGroup(
                FlyWheel.INSTANCE.onAuto,
                new SequentialGroup(
                        new FollowPath(launchPath),
                        runIntake,
                        shoot1,
                        new Delay(1),
                        runUptake,
                        new Delay(2),
                        shoot2,
                        new Delay(1),
                        runUptake,
                        new Delay(2),
                        shoot3,
                        new Delay(1),
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
        if (Turret.INSTANCE.getIndex(limelight) != 0 && Turret.INSTANCE.getIndex(limelight) != 20 && Turret.INSTANCE.getIndex(limelight) != 24 && lastIndex == 0 && Turret.INSTANCE.getIndex(limelight) != 24) {
            lastIndex = Turret.INSTANCE.getIndex(limelight);
            telemetry.addData("index", lastIndex);
        }
        telemetry.update();
    }

    public static Limelight3A limelight = null;

    @Override
    public void onStop() {

    }



}
