package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

import dev.nextftc.control.KineticState;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@TeleOp(name = "FlyWheelRegressionCode")

public class FlyWheelRegressionCode extends NextFTCOpMode {

    ServoEx hood = new ServoEx("hood");
    MotorEx intake = new MotorEx("intake");
    CRServoEx uptake = new CRServoEx("uptake");


    private static Limelight3A limelight;



    public FlyWheelRegressionCode() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(FlyWheel.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE)
        );
    }

    @Override
    public void onStartButtonPressed() {
        FlyWheel.INSTANCE.isStarted = true;
        Turret.INSTANCE.opModeIsStarted = true;

    }

    @Override
    public void onUpdate() {

        Turret.INSTANCE.relocalizationUpdate(limelight, telemetry);
        Turret.INSTANCE.turretMovement(false);

        if(gamepad1.y){
            FlyWheel.INSTANCE.FlyWheelControl.setGoal(new KineticState(0, FlyWheel.INSTANCE.FlyWheelControl.getGoal().getVelocity()+100));
        } else if (gamepad1.b){
            FlyWheel.INSTANCE.FlyWheelControl.setGoal(new KineticState(0, FlyWheel.INSTANCE.FlyWheelControl.getGoal().getVelocity()+10));
        } else if (gamepad1.a){
            FlyWheel.INSTANCE.FlyWheelControl.setGoal(new KineticState(0, FlyWheel.INSTANCE.FlyWheelControl.getGoal().getVelocity()-100));
        } else if (gamepad1.x){
            FlyWheel.INSTANCE.FlyWheelControl.setGoal(new KineticState(0, FlyWheel.INSTANCE.FlyWheelControl.getGoal().getVelocity()-10));
        }

        if(gamepad1.dpad_up){
            hood.setPosition(hood.getPosition()+.1);
        } else if(gamepad1.dpad_right){
            hood.setPosition(hood.getPosition()+.05);
        } else if(gamepad1.dpad_down){
            hood.setPosition(hood.getPosition()-.1);
        } else if(gamepad1.dpad_left){
            hood.setPosition(hood.getPosition()-.05);
        }

        if(gamepad1.right_bumper){
            Turret.INSTANCE.resetButton();
        }

        if(gamepad1.left_bumper){
            intake.setPower(-1);
            uptake.setPower(1);
        }else{
            intake.setPower(0);
            uptake.setPower(0);
        }

        telemetry.addData("Distance from goal: ", Turret.INSTANCE.distanceOffsetBlue);
        telemetry.addData("Hood Position: ", hood.getPosition());
        telemetry.addData("leftFlyWheel Speed: ", FlyWheel.INSTANCE.topFW.getVelocity());
        telemetry.addData("rightFlyWheel Speed: ", FlyWheel.INSTANCE.bottomFW.getVelocity());
        telemetry.addData("FlyWheel Goal: ", FlyWheel.INSTANCE.FlyWheelControl.getGoal());
        telemetry.update();
    }

    @Override
    public void onInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
    }

    @Override
    public void onStop() {
    }
}
