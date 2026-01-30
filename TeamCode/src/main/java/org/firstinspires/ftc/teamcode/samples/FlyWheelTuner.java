package org.firstinspires.ftc.teamcode.samples;

import static org.firstinspires.ftc.teamcode.NewTeleop.limelight;
import static java.lang.Math.abs;

import dev.nextftc.control.KineticState;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.ServoEx;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

@TeleOp(name = "runFlyWheelPIDs")

public class FlyWheelTuner extends NextFTCOpMode {

    ServoEx hood = new ServoEx("hood");

    public FlyWheelTuner() {
        addComponents(
                new SubsystemComponent(FlyWheel.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE)
        );
    }

    @Override
    public void onStartButtonPressed() {
        FlyWheel.INSTANCE.FlyWheelControl.setGoal(new KineticState(0));
    }

    @Override
    public void onUpdate() {
        if (gamepad1.b){
            FlyWheel.INSTANCE.FlyWheelControl.setGoal(new KineticState(FlyWheel.INSTANCE.FlyWheelControl.getGoal().getPosition(),FlyWheel.INSTANCE.FlyWheelControl.getGoal().getVelocity() + 10));
        } else if (gamepad1.y) {
            FlyWheel.INSTANCE.FlyWheelControl.setGoal(new KineticState(FlyWheel.INSTANCE.FlyWheelControl.getGoal().getPosition(),FlyWheel.INSTANCE.FlyWheelControl.getGoal().getVelocity() + 100));
        } else if (gamepad1.a) {
            FlyWheel.INSTANCE.FlyWheelControl.setGoal(new KineticState(FlyWheel.INSTANCE.FlyWheelControl.getGoal().getPosition(),FlyWheel.INSTANCE.FlyWheelControl.getGoal().getVelocity() - 100));
        } else if (gamepad1.x) {
            FlyWheel.INSTANCE.FlyWheelControl.setGoal(new KineticState(FlyWheel.INSTANCE.FlyWheelControl.getGoal().getPosition(),FlyWheel.INSTANCE.FlyWheelControl.getGoal().getVelocity() - 10));
        }

        if (gamepad1.dpad_down) {
            hood.setPosition(.12);
        } else if (gamepad1.dpad_up) {
            hood.setPosition(.065);
        } else if (gamepad1.dpad_right) {
            hood.setPosition(.13);
        } else if (gamepad1.dpad_left) {
            hood.setPosition(.14);
        }

        telemetry.addData("flywheel goal: ", FlyWheel.INSTANCE.FlyWheelControl.getGoal());
        telemetry.addData("hood pos:", hood.getPosition());
        telemetry.update();
    }

    @Override
    public void onInit() {
    }

    @Override
    public void onStop() {
    }
}
