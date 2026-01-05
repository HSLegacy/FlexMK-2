package org.firstinspires.ftc.teamcode.samples;

import static java.lang.Math.abs;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;

@TeleOp(name = "sensor Test")

public class limitSwitchTest extends NextFTCOpMode {
    private DigitalChannel limitSwitch = null;

    @Override
    public void onStartButtonPressed() {
    }

    @Override
    public void onUpdate() {
            telemetry.addData("switchyPoo: ", limitSwitch.getState());
            telemetry.update();
    }

    @Override
    public void onInit() {
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
    }

    @Override
    public void onStop() {
    }
}
