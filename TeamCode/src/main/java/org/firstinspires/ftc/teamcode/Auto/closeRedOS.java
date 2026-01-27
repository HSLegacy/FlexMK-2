package org.firstinspires.ftc.teamcode.Auto;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

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
import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subSystems.Spindexer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

import java.util.Timer;

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
import dev.nextftc.hardware.powerable.SetPower;

@Autonomous(name = "closeRedOS")

public class closeRedOS extends NextFTCOpMode {
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
    private final Pose launchPose = new Pose(55, 125, Math.toRadians(160));
    private final Pose parkPose = new Pose(55,130, Math.toRadians(0));

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
    public closeRedOS() {
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
        Spindexer.INSTANCE.intakePos1.schedule();
        buildPaths();
        follower().setStartingPose(startPose);
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                new FollowPath(launchPath),
                fire,
                new Delay(4),
                new FollowPath(parkPath)
        );
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
        Turret.INSTANCE.flyWheelGoal = 1180;
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
