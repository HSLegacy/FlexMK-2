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

    public ControlSystem spindexerControl = ControlSystem.builder()
            .posPid(0.015,0, 0.00001)
            .build();


    public final Command intakePos1 = new RunToPosition(spindexerControl, 0).requires(this).named("intakePos1");
    public final Command intakePos2 = new RunToPosition(spindexerControl, 171).requires(this).named("intakePos2");
    public final Command intakePos3 = new RunToPosition(spindexerControl, 358).requires(this).named("intakePos3");
    public final Command outakePos1 = new RunToPosition(spindexerControl, 89).requires(this).named("outtakePos1"); //fire intakePos1
    public final Command outakePos2 = new RunToPosition(spindexerControl, 267).requires(this).named("outtakePos2");
    public final Command outakePos3 = new RunToPosition(spindexerControl, 439).requires(this).named("outtakePos3");

    private boolean isStarted = false;

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {

        telemetryManager.getTelemetry().addData("spindexer pos", spindexer.getState().toString());
        telemetryManager.getTelemetry().update();
        manager.update();
        spindexer.setPower((spindexerControl.calculate(spindexer.getState()))*0.4);

    }
}