package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        //Spindexer.INSTANCE.intakePos3.schedule();
        telemetry.update();
    }

    @Override
    public void onInit() {
    }

    @Override
    public void onStop() {
    }
}
