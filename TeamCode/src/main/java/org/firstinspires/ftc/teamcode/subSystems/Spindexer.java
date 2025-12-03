package org.firstinspires.ftc.teamcode.subSystems;




import static java.lang.Math.abs;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;



public class Spindexer implements Subsystem {

    public static final Spindexer INSTANCE = new Spindexer();
    PanelsTelemetry telemetryManager = PanelsTelemetry.INSTANCE;
    GraphManager manager = PanelsGraph.INSTANCE.getManager();

    CRServoEx rUptake = new CRServoEx("rUptake");
    CRServoEx lUptake = new CRServoEx("lUptake");
    private Spindexer() {
    }

    public MotorEx spindexer = new MotorEx("spindexer");

    private ControlSystem spindexerControl = ControlSystem.builder()
            .posPid(0.007,0, 0.00015)
            .build();


    public final Command intakePos1 = new RunToPosition(spindexerControl, 0).requires(this).named("intakePos1");
    public final Command intakePos2 = new RunToPosition(spindexerControl, 177).requires(this).named("intakePos2");
    public final Command intakePos3 = new RunToPosition(spindexerControl, 359).requires(this).named("intakePos3");
    public final Command outakePos1 = new RunToPosition(spindexerControl, 82).requires(this).named("outtakePos1");
    public final Command outakePos2 = new RunToPosition(spindexerControl, 267).requires(this).named("outtakePos2");
    public final Command outakePos3 = new RunToPosition(spindexerControl, 446).requires(this).named("outtakePos3");

    private boolean isStarted = false;

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {

        telemetryManager.getTelemetry().addData("spindexer pos", spindexer.getState().toString());
        telemetryManager.getTelemetry().update();
        manager.update();

        spindexer.setPower(spindexerControl.calculate(spindexer.getState()));

        if(spindexer.getVelocity() > 1){
            lUptake.setPower(-1);
            rUptake.setPower(-1);
        } else if(spindexer.getVelocity() < -1){
            lUptake.setPower(1);
            rUptake.setPower(1);
        } else {
            lUptake.setPower(0);
            rUptake.setPower(0);
        }
    }
}