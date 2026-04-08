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
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import java.util.Timer;

@Autonomous(name = "farBlue")

public class farBlue extends NextFTCOpMode {
    MotorEx intake = new MotorEx("intake");
    CRServoEx uptake = new CRServoEx("uptake");
    ServoEx gate = new ServoEx("door");
    DigitalChannel limitSwitch = null;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;



    private final Pose startPose = new Pose(57, 5, Math.toRadians(180));
    private final Pose launchPose = new Pose(52, 18, Math.toRadians(180));
    private final Pose subStationPickUpPose1 = new Pose(15, 15, Math.toRadians(-157));
    private final Pose subStationPickUpPose2 = new Pose(14, 9, Math.toRadians(-157));
    private final Pose cyclePose = new Pose(19, 11, Math.toRadians(-160));
    private final Pose cyclePose2 = new Pose(22, 11, Math.toRadians(180));
    private final Pose cyclePose3 = new Pose(16.5, 11, Math.toRadians(180));
    private final Pose parkPose = new Pose(43,17, Math.toRadians(180));

    private final Pose spike3spot1 = new Pose(35, 35, Math.toRadians(180));
    private final Pose spike3spot2 = new Pose(15, 35, Math.toRadians(180));

    public PathChain launchPath, parkPath, sub1Path, sub2Path, spike31, spike32, launchPath2, cyclePath, launchPath3, cyclePath2, cyclePath3;

    public void buildPaths() {
        sub1Path = follower().pathBuilder()
                .addPath(new BezierLine(startPose, subStationPickUpPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), subStationPickUpPose1.getHeading())
                .build();
        sub2Path = follower().pathBuilder()
                .addPath(new BezierLine(subStationPickUpPose1, subStationPickUpPose2))
                .setLinearHeadingInterpolation(subStationPickUpPose1.getHeading(), subStationPickUpPose2.getHeading())
                .build();
        launchPath = follower().pathBuilder()
                .addPath(new BezierLine(subStationPickUpPose2, launchPose))
                .setLinearHeadingInterpolation(subStationPickUpPose2.getHeading(), launchPose.getHeading())
                .build();
        cyclePath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, cyclePose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), cyclePose.getHeading())
                .build();
        cyclePath2 = follower().pathBuilder()
                .addPath(new BezierLine(cyclePose, cyclePose2))
                .setLinearHeadingInterpolation(cyclePose.getHeading(), cyclePose2.getHeading())
                .build();
        cyclePath3 = follower().pathBuilder()
                .addPath(new BezierLine(cyclePose2, cyclePose3))
                .setLinearHeadingInterpolation(cyclePose2.getHeading(), cyclePose3.getHeading())
                .build();
        launchPath2 = follower().pathBuilder()
                .addPath(new BezierLine(cyclePose3, launchPose))
                .setLinearHeadingInterpolation(cyclePose3.getHeading(), launchPose.getHeading())
                .build();
        parkPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, parkPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), parkPose.getHeading())
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
                Turret.INSTANCE.targetPoseBlue = new Pose(2, 139);
            });
    public Command setOldTurretPose = new LambdaCommand()
            .setStart(() -> {
                Turret.INSTANCE.targetPoseBlue = new Pose(5, 142);
            });

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

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        FlyWheel.INSTANCE.off.schedule();
        buildPaths();
        follower().setStartingPose(startPose);
        Turret.INSTANCE.turretMotor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Turret.INSTANCE.limelight = limelight;
        Turret.INSTANCE.telemetry = telemetry;
        Turret.INSTANCE.limitSwitch = limitSwitch;



    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                setNewTurretPose,
                relocalize,
                runIntake,
                openGate,
                new Delay(3),
                closeGate,
                new FollowPath(sub1Path),
                new FollowPath(sub2Path),
                new Delay(.5),
                new FollowPath(launchPath),
                relocalize,
                openGate,
                new Delay(2),
                closeGate,
                new FollowPath(cyclePath),
                new FollowPath(cyclePath2),
                new FollowPath(cyclePath3),
                new FollowPath(launchPath2),
                relocalize,
                openGate,
                new Delay(2),
                closeGate,
                new FollowPath(cyclePath),
                new FollowPath(cyclePath2),
                new FollowPath(cyclePath3),
                new FollowPath(launchPath2),
                relocalize,
                openGate,
                new Delay(2),
                closeGate,
                new FollowPath(cyclePath),
                new FollowPath(cyclePath2),
                new FollowPath(cyclePath3),
                new FollowPath(launchPath2),
                relocalize,
                openGate,
                new Delay(2),
                closeGate,
                new FollowPath(parkPath)
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