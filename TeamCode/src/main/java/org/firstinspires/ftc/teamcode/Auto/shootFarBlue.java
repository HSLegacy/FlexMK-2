package org.firstinspires.ftc.teamcode.Auto;
import org.firstinspires.ftc.teamcode.subSystems.Spindexer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;
import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;

import static java.lang.Math.abs;

import dev.nextftc.control.KineticState;
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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

@Autonomous(name = "shootFarBlue")

public class shootFarBlue extends NextFTCOpMode {
    double flyWheelGoal = 1100;
    boolean lockedOn = false;
    CRServoEx intake = new CRServoEx("intake");
    CRServoEx intake2 = new CRServoEx("intake2");
    CRServoEx upTakeWheel = new CRServoEx("upTakeWheel");
    ServoEx gate = new ServoEx("gate");

    Turret turret = Turret.INSTANCE;
    private DigitalChannel limitSwitch = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private int lastIndex = 0;
    private final Pose startPose = new Pose(52, 9, Math.toRadians(180));
    private final Pose launchPose = new Pose(52, 15, Math.toRadians(180));
    private final Pose subStationPickUpPose1 = new Pose(10.5, 20, Math.toRadians(200));
    private final Pose subStationPickUpPose2 = new Pose(9.5, 11, Math.toRadians(210));
    private final Pose spike3spot1 = new Pose(35, 35, Math.toRadians(180));
    private final Pose spike3spot2 = new Pose(15, 35, Math.toRadians(180));
    private final Pose parkPose = new Pose(50,25, Math.toRadians(180));

    public PathChain launchPath, parkPath, sub1Path, sub2Path, spike31, spike32, launchPath2;

    public void buildPaths() {
        sub1Path = follower().pathBuilder()
                .addPath(new BezierLine(startPose, subStationPickUpPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), subStationPickUpPose1.getHeading())
                .build();
        sub2Path = follower().pathBuilder()
                .addPath(new BezierLine(subStationPickUpPose1, subStationPickUpPose2))
                .setLinearHeadingInterpolation(subStationPickUpPose1.getHeading(), subStationPickUpPose2.getHeading())
                .build();
        spike31 = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, spike3spot1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), spike3spot1.getHeading())
                .build();
        spike32 = follower().pathBuilder()
                .addPath(new BezierLine(spike3spot1, spike3spot2))
                .setLinearHeadingInterpolation(spike3spot1.getHeading(), spike3spot2.getHeading())
                .build();
        launchPath = follower().pathBuilder()
                .addPath(new BezierLine(subStationPickUpPose2, launchPose))
                .setLinearHeadingInterpolation(subStationPickUpPose2.getHeading(), launchPose.getHeading())
                .addParametricCallback(.2, closeGate)
                .addParametricCallback(.6, fire)
                .build();
        launchPath2 = follower().pathBuilder()
                .addPath(new BezierLine(spike3spot2, launchPose))
                .setLinearHeadingInterpolation(spike3spot2.getHeading(), launchPose.getHeading())
                .addParametricCallback(.2, closeGate)
                .addParametricCallback(.7, fire)
                .build();
        parkPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, parkPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), parkPose.getHeading())
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

    public shootFarBlue() {
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
                new Delay(1),
                fire,
                new Delay(4),
                openGate,
                runIntake,
                new FollowPath(sub1Path),
                new FollowPath(sub2Path, true, .6),
                new Delay(.5),
                new FollowPath(launchPath),
                new Delay(3),
                openGate,
                runIntake,
                new FollowPath(spike31),
                new FollowPath(spike32, true, .7),
                new FollowPath(launchPath2),
                new Delay(4),
                new FollowPath(parkPath)
        );
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
        turret.flyWheelGoal = 1200;
        turret.lastHeading = 0;
        turret.hood.setPosition(.12);
        turret.turretControl.setGoal(new KineticState(-1000));
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
