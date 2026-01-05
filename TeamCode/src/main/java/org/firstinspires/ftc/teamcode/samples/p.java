package org.firstinspires.ftc.teamcode.samples;

import static java.lang.Math.abs;

import dev.nextftc.ftc.NextFTCOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "c")

public class p extends NextFTCOpMode {
    private NormalizedColorSensor p = null;

    @Override
    public void onStartButtonPressed() {
    }

    @Override
    public void onUpdate() {
        NormalizedRGBA colors = p.getNormalizedColors();
            telemetry.addData("r: ", colors.red);
            telemetry.addData("g: ", colors.green);
            telemetry.addData("b: ", colors.blue);
            telemetry.update();
    }

    @Override
    public void onInit() {
        p = hardwareMap.get(NormalizedColorSensor.class, "c");
    }

    @Override
    public void onStop() {
    }
}