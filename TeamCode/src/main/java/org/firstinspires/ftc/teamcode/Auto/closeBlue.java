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
import com.qualcomm.robotcore.hardware.DigitalChannel;

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
    private DigitalChannel limitSwitch = null;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(22, 123, Math.toRadians(90));
    private final Pose cameraPose = new Pose(60, 95, Math.toRadians(90));
    private final Pose launchPose = new Pose(60, 85, Math.toRadians(145));
    private final Pose pickUp1 = new Pose(37.5, 85.5, Math.toRadians(180));
    private final Pose pickUp3 = new Pose(17, 85.5, Math.toRadians(180));
    private final Pose launchPose2 = new Pose(60, 85, Math.toRadians(145));
    private final Pose spike2Spot1 = new Pose(37.5, 60.5, Math.toRadians(180));
    private final Pose spike2Spot2 = new Pose(14, 60.5, Math.toRadians(180));
    private final Pose launchPose3 = new Pose(60, 85, Math.toRadians(138));
    private final Pose parkPose = new Pose(55,130, Math.toRadians(180));

    public PathChain cameraPath, launchPath, pickPath1, pickPath2, launchPath2, parkPath, spike21, spike22, launchPath3;

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
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickUp1.getHeading())
                .build();
        pickPath2 = follower().pathBuilder()
                .addPath(new BezierLine(pickUp1, pickUp3))
                .setLinearHeadingInterpolation(pickUp1.getHeading(), pickUp3.getHeading())
                .build();
        launchPath2 = follower().pathBuilder()
                .addPath(new BezierLine(pickUp3, launchPose2))
                .setLinearHeadingInterpolation(pickUp3.getHeading(), launchPose2.getHeading())
                .setBrakingStrength(1)
                .build();
        spike21 = follower().pathBuilder()
                .addPath(new BezierLine(launchPose2,spike2Spot1))
                .setLinearHeadingInterpolation(launchPose2.getHeading(), spike2Spot1.getHeading())
                .build();
        spike22 = follower().pathBuilder()
                .addPath(new BezierLine(spike2Spot1,spike2Spot2))
                .setLinearHeadingInterpolation(spike2Spot1.getHeading(), spike2Spot2.getHeading())
                .setBrakingStrength(1)
                .build();
        launchPath3 = follower().pathBuilder()
                .addPath(new BezierLine(spike2Spot2,launchPose3))
                .setLinearHeadingInterpolation(spike2Spot2.getHeading(), launchPose3.getHeading())
                .build();
        parkPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, parkPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public Command runIntake = new SetPower(intake, -1);

    public Command fire = new LambdaCommand()
            .setStart(() -> {
                telemetry.addData("started", "it defeninitlky did the thing");
                lUptake.setPower(-1);
                rUptake.setPower(1);
                Spindexer.INSTANCE.firingPosition.schedule();
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
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
        FlyWheel.INSTANCE.off.schedule();
        buildPaths();
        follower().setStartingPose(startPose);
    }

    private Command autonomousRoutine() {
            return new SequentialGroup(
                new FollowPath(cameraPath),
                findIndex,
                new FollowPath(launchPath),
                fire,
                new Delay(4),
                runIntake,
                new FollowPath(pickPath1),
                new FollowPath(pickPath2, true, .6),
                new FollowPath(launchPath2),
                fire,
                new Delay(4),
                new FollowPath(spike21),
                new FollowPath(spike22, true, .6),
                new FollowPath(launchPath3),
                fire,
                new Delay(4),
                new FollowPath(parkPath)
        );
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
        Turret.INSTANCE.flyWheelGoal = 1200;
        Spindexer.INSTANCE.spindexer.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Spindexer.INSTANCE.isStarted = true;
    }


    @Override
    public void onUpdate() {
        if (Turret.INSTANCE.getIndex(limelight) != 0 && Turret.INSTANCE.getIndex(limelight) != 20 && Turret.INSTANCE.getIndex(limelight) != 24 && lastIndex == 0 && Turret.INSTANCE.getIndex(limelight) != 24) {
            lastIndex = Turret.INSTANCE.getIndex(limelight);
            telemetry.addData("index", lastIndex);
        }

        if(!limitSwitch.getState() && Spindexer.INSTANCE.spindexerControl.getGoal().getPosition() != -45){
            Spindexer.INSTANCE.spindexer.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Spindexer.INSTANCE.intakePosition.schedule();
            lUptake.setPower(0);
            rUptake.setPower(0);
        }

        FlyWheel.INSTANCE.setGoal(Turret.INSTANCE.flyWheelGoal);
        Turret.INSTANCE.lockOnUpdate(limelight, telemetry);

        telemetry.addData("spindexer Pos", Spindexer.INSTANCE.spindexer.getCurrentPosition());
        telemetry.addData("spindexer Goal", Spindexer.INSTANCE.spindexerControl.getGoal().getPosition());
        telemetry.addData("Flywheel Goal", Turret.INSTANCE.flyWheelGoal);
        telemetry.update();
    }

    public static Limelight3A limelight = null;

    @Override
    public void onStop() {

    }


}
