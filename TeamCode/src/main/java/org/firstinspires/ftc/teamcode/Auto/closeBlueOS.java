package org.firstinspires.ftc.teamcode.Auto;
import org.firstinspires.ftc.teamcode.subSystems.Spindexer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;
import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;


import static java.lang.Math.abs;

import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
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

@Autonomous(name = "closeBlueOS")

public class closeBlueOS extends NextFTCOpMode {
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
    private final Pose launchPose = new Pose(55, 125, Math.toRadians(180));

    public PathChain launchPath;

    public void buildPaths() {
        launchPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .addParametricCallback(.3, fire)
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
    public closeBlueOS() {
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
                closeGate,
                new FollowPath(launchPath)
        );
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
        turret.flyWheelGoal = 1200;
        turret.lastHeading = 180;
        turret.hood.setPosition(.12);
        turret.turretControl.setGoal(new KineticState(-500));
        Spindexer.INSTANCE.spindexer.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Spindexer.INSTANCE.isStarted = true;
        FlyWheel.INSTANCE.isStarted = true;
        turret.turretPower = true;
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
