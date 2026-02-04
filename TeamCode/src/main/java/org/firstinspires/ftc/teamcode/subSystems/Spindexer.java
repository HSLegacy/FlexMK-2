package org.firstinspires.ftc.teamcode.subSystems;




import static java.lang.Math.abs;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

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
            .posPid(0.01,0, 0)
            .build();


    //Old spindexer pos stuff
    public final Command intakePos1 = new RunToPosition(spindexerControl, 0).requires(this).named("intakePos1");
    public final Command intakePos2 = new RunToPosition(spindexerControl, 171).requires(this).named("intakePos2");
    public final Command intakePos3 = new RunToPosition(spindexerControl, 358).requires(this).named("intakePos3");
    public final Command outakePos1 = new RunToPosition(spindexerControl, 84).requires(this).named("outtakePos1"); //fire intakePos1
    public final Command outakePos2 = new RunToPosition(spindexerControl, 267).requires(this).named("outtakePos2");
    public final Command outakePos3 = new RunToPosition(spindexerControl, 439).requires(this).named("outtakePos3");
    public final Command autoOutakePos4 = new RunToPosition(spindexerControl, -85).requires(this).named("outtakePos3");
    public final Command autoIntakePos4 = new RunToPosition(spindexerControl, -25).requires(this).named("outtakePos3");
    public final Command telopOffset = new RunToPosition(spindexerControl, -20).requires(this).named("outtakePos3");

    //New Spindexer pos stuff
    public final Command intakePosition = new RunToPosition(spindexerControl, 0, 0).requires(this).named("intakePos1");
    public final Command firingPosition = new RunToPosition(spindexerControl, -1110, 0).requires(this).named("intakePos1");
    public boolean isStarted = false;

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {
        telemetryManager.getTelemetry().addData("spindexer pos", spindexer.getState().toString());
        telemetryManager.getTelemetry().addData("spindexer goal", spindexerControl.getGoal());
        telemetryManager.getTelemetry().update();
        manager.update();
        if(isStarted){
            spindexer.setPower((spindexerControl.calculate(spindexer.getState())));
        }

    }

}