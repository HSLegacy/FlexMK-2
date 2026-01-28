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
import org.firstinspires.ftc.teamcode.subSystems.Turret;

@TeleOp(name = "turretTuner")

public class turretTuner extends NextFTCOpMode {


    public turretTuner() {
        addComponents(
                new SubsystemComponent(Turret.INSTANCE)
        );
    }

    @Override
    public void onStartButtonPressed() {
    }

    @Override
    public void onUpdate() {
        if(gamepad1.b){
            Turret.INSTANCE.goal150Turret.schedule();
        } else {
            Turret.INSTANCE.goal0Turret.schedule();
        }
        telemetry.addData("Turret Pos: ", Turret.INSTANCE.turretMotor.getCurrentPosition());
        telemetry.addData("Turret Goal: ", Turret.INSTANCE.turretControl.getGoal());
        telemetry.update();
    }

    @Override
    public void onInit() {
    }

    @Override
    public void onStop() {
    }
}
