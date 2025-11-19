package org.firstinspires.ftc.teamcode.samples;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

import dev.nextftc.ftc.NextFTCOpMode;

@Disabled
@Autonomous(name = "dbTest")
public class dbTest extends NextFTCOpMode {
    Limelight3A limelight;
    private ControlSystem turret;
    private DcMotorEx yLinear;

    @Override
    public void onInit() {
        yLinear = hardwareMap.get(DcMotorEx.class, "yLinear");
        turret = ControlSystem.builder()
                .posPid(0.1, 0.0, 0.0)
                .build();
        turret.setGoal(new KineticState(0.0));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
    }


    @Override
    public void onUpdate() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                telemetry.addData("ty", result.getTy());
                yLinear.setPower(turret.calculate(
                        new KineticState(576d / result.getTy())
                ));
            }
        }
    }
}