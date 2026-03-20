package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subSystems.Spindexer;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "FlyWheelTuner")

public class flyWheelTunner extends NextFTCOpMode {


    public flyWheelTunner() {
        addComponents(
                new SubsystemComponent(FlyWheel.INSTANCE)
        );
    }

    @Override
    public void onStartButtonPressed() {
        FlyWheel.INSTANCE.isStarted = true;
    }

    @Override
    public void onUpdate() {
        if(gamepad1.b){
            FlyWheel.INSTANCE.on.schedule();
        } else {
            FlyWheel.INSTANCE.off.schedule();
        }
        telemetry.addData("leftFlyWheel Speed: ", FlyWheel.INSTANCE.topFW.getVelocity());
        telemetry.addData("rightFlyWheel Speed: ", FlyWheel.INSTANCE.bottomFW.getVelocity());
        telemetry.addData("FlyWheel Goal: ", FlyWheel.INSTANCE.FlyWheelControl.getGoal());
        telemetry.update();
    }

    @Override
    public void onInit() {
    }

    @Override
    public void onStop() {
    }
}
