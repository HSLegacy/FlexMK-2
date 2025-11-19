package org.firstinspires.ftc.teamcode.samples;

import static java.lang.Math.abs;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;

@Configurable
@Disabled
@TeleOp(name = "runFlyWheelPIDs")

public class FlyWheelTuner extends NextFTCOpMode {


    public FlyWheelTuner() {
        addComponents(
                new SubsystemComponent(FlyWheel.INSTANCE)
        );
    }

    @Override
    public void onStartButtonPressed() {
    }

    @Override
    public void onUpdate() {
        FlyWheel.INSTANCE.on.schedule();
        //FlyWheel.INSTANCE.on1.schedule();
        telemetry.update();
    }

    @Override
    public void onInit() {
    }

    @Override
    public void onStop() {
    }
}
