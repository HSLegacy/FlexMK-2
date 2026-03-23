package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "spindexerTuner")

public class spindexerTuner extends NextFTCOpMode {


    public spindexerTuner() {
        addComponents(
                new SubsystemComponent(Spindexer.INSTANCE)
        );
    }

    @Override
    public void onStartButtonPressed() {
        Spindexer.INSTANCE.spindexer.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void onUpdate() {
        if(gamepad1.b){
            Spindexer.INSTANCE.firingPosition.schedule();
        } else {
            Spindexer.INSTANCE.intakePosition.schedule();
        }
        telemetry.addData("Spindexer Pos: ", Spindexer.INSTANCE.spindexer.getCurrentPosition());
        telemetry.addData("Spindexer Goal: ", Spindexer.INSTANCE.spindexerControl.getGoal());
        telemetry.update();
    }

    @Override
    public void onInit() {
    }

    @Override
    public void onStop() {
    }
}
