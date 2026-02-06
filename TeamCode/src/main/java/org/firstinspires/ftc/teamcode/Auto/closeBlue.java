package org.firstinspires.ftc.teamcode.Auto;
import org.firstinspires.ftc.teamcode.subSystems.Spindexer;
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
import dev.nextftc.hardware.impl.ServoEx;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
    CRServoEx intake2 = new CRServoEx("intake2");
    CRServoEx upTakeWheel = new CRServoEx("upTakeWheel");
    ServoEx gate = new ServoEx("gate");

    Turret turret = Turret.INSTANCE;

    LLResultTypes.FiducialResult lastResult = null;
    private int lastIndex = 0;
    private DigitalChannel limitSwitch = null;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(22, 123, Math.toRadians(180));
    private final Pose launchPose = new Pose(55, 85, Math.toRadians(180));
    private final Pose spike2Spot1 = new Pose(39, 59, Math.toRadians(180));
    private final Pose spike2Spot2 = new Pose(18, 67, Math.toRadians(180));
    private final Pose spike1Spot1 = new Pose(37.5, 85.5, Math.toRadians(180));
    private final Pose spike1Spot2 = new Pose(17, 85.5, Math.toRadians(180));
    private final Pose launch2MidPoint = new Pose(40, 63.5, Math.toRadians(180));
    private final Pose launchPose2 = new Pose(55, 85, Math.toRadians(180));
    private final Pose launchPose31 = new Pose(55, 90, Math.toRadians(180));
    private final Pose launchPose32 = new Pose(55, 105, Math.toRadians(180));
    private final Pose spike3Spot1 = new Pose(38, 37, Math.toRadians(180));
    private final Pose spike3Spot2 = new Pose(18, 37, Math.toRadians(180));
    private final Pose parkPose = new Pose(55,130, Math.toRadians(180));

    public PathChain launchPath, spike11, spike12, launchPath2, parkPath, spike21, spike22, launchPath3, spike31, spike32, launchPath41, launchPath42, fillerPath;

    public void buildPaths() {
        launchPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .addParametricCallback(.3, fire)
                .build();
        spike21 = follower().pathBuilder()
                .addPath(new BezierLine(launchPose2,spike2Spot1))
                .setLinearHeadingInterpolation(launchPose2.getHeading(), spike2Spot1.getHeading())
                .build();
        spike22 = follower().pathBuilder()
                .addPath(new BezierLine(spike2Spot1,spike2Spot2))
                .setLinearHeadingInterpolation(spike2Spot1.getHeading(), spike2Spot2.getHeading())
                .build();
        spike11 = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, spike1Spot1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), spike1Spot1.getHeading())
                .build();
        spike12 = follower().pathBuilder()
                .addPath(new BezierLine(spike1Spot1, spike1Spot2))
                .setLinearHeadingInterpolation(spike1Spot1.getHeading(), spike1Spot2.getHeading())
                .build();
        launchPath2 = follower().pathBuilder()
                .addPath(new BezierCurve(spike2Spot2, launch2MidPoint, launchPose2))
                .setLinearHeadingInterpolation(spike1Spot2.getHeading(), launchPose2.getHeading())
                .addParametricCallback(.2, closeGate)
                .addParametricCallback(.8, fire)
                .build();
        launchPath3 = follower().pathBuilder()
                .addPath(new BezierLine(spike1Spot2,launchPose))
                .setLinearHeadingInterpolation(spike2Spot2.getHeading(), launchPose.getHeading())
                .addParametricCallback(.1, closeGate)
                .addParametricCallback(.6, fire)
                .build();
        spike31 = follower().pathBuilder()
                .addPath(new BezierLine(launchPose32,spike3Spot1))
                .setLinearHeadingInterpolation(launchPose32.getHeading(), spike3Spot1.getHeading())
                .build();
        spike32 = follower().pathBuilder()
                .addPath(new BezierLine(spike3Spot1,spike3Spot2))
                .setLinearHeadingInterpolation(spike3Spot1.getHeading(), spike3Spot2.getHeading())
                .build();
        launchPath41 = follower().pathBuilder()
                .addPath(new BezierLine(spike3Spot2, launchPose32))
                .setLinearHeadingInterpolation(spike3Spot2.getHeading(), launchPose32.getHeading())
                .addParametricCallback(.5, closeGate)
                .addParametricCallback(.8, fire)
                .build();
        parkPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose32, parkPose))
                .setLinearHeadingInterpolation(launchPose32.getHeading(), parkPose.getHeading())
                .build();
        fillerPath = follower().pathBuilder()
                .addPath(new BezierLine(PedroComponent.follower().getPose(), PedroComponent.follower().getPose()))
                .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), PedroComponent.follower().getHeading())
                .build();
    }

    public Command runIntake = new LambdaCommand()
            .setStart(() -> {
                intake.setPower(1);
                intake2.setPower(-1);
                gate.setPosition(.97);
            });

    public Command fire = new LambdaCommand()
            .setStart(() -> {
                upTakeWheel.setPower(1);
                gate.setPosition(.75);
                Spindexer.INSTANCE.firingPosition.schedule();
            });
    public Command openGate = new LambdaCommand()
            .setStart(() -> {
                gate.setPosition(.97);
            });
    public Command closeGate = new LambdaCommand()
            .setStart(() -> {
                gate.setPosition(.75);
                intake.setPower(-1);
                intake2.setPower(1);
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
        Spindexer.INSTANCE.intakePosition.schedule();
        Spindexer.INSTANCE.spindexer.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.turretMotor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.turretControl.setGoal(new KineticState(0));
    }

    private Command autonomousRoutine() {
            return new SequentialGroup(
                new FollowPath(launchPath),
                new Delay(2),
                runIntake,
                new FollowPath(spike21, false),
                new FollowPath(spike22, false, 0.8),
                new FollowPath(launchPath2),
                new Delay(3.5),
                openGate,
                runIntake,
                new FollowPath(spike12, false, 0.8),
                new FollowPath(launchPath3),
                fire,
                new Delay(3.5),
                openGate,
                runIntake,
                new FollowPath(spike31, false),
                new FollowPath(spike32, false, 0.8),
                new FollowPath(launchPath41),
                fire,
                new Delay(3.5),
                new FollowPath(parkPath)
        );
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
        turret.flyWheelGoal = 1200;
        turret.lastHeading = 115;
        turret.hood.setPosition(.12);
        turret.turretControl.setGoal(new KineticState(-500));
        Spindexer.INSTANCE.spindexer.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Spindexer.INSTANCE.isStarted = true;
    }


    @Override
    public void onUpdate() {
        if (Turret.INSTANCE.getIndex(limelight) != 0 && Turret.INSTANCE.getIndex(limelight) != 20 && Turret.INSTANCE.getIndex(limelight) != 24 && lastIndex == 0 && Turret.INSTANCE.getIndex(limelight) != 24) {
            lastIndex = Turret.INSTANCE.getIndex(limelight);
            telemetry.addData("index", lastIndex);
        }

        if(Spindexer.INSTANCE.spindexer.getMotor().getCurrentPosition() < -1095 && Spindexer.INSTANCE.spindexer.getMotor().getCurrentPosition() > -1120){
            Spindexer.INSTANCE.spindexerControl.setGoal(new KineticState(180));
        }


        FlyWheel.INSTANCE.setGoal(Turret.INSTANCE.flyWheelGoal);

        turret.lockOnUpdate(limelight, telemetry);
        turret.lockOnTurretBlue(limelight, telemetry);

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
