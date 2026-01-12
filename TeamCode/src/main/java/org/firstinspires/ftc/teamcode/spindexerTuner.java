package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import static dev.nextftc.bindings.Bindings.button;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subSystems.Spindexer;

@TeleOp(name = "spindexerTuner")

public class spindexerTuner extends NextFTCOpMode {


    public spindexerTuner() {
        addComponents(
                new SubsystemComponent(Spindexer.INSTANCE)
        );
    }

    @Override
    public void onStartButtonPressed() {
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
