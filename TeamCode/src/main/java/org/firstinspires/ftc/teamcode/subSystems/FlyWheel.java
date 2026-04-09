package org.firstinspires.ftc.teamcode.subSystems;




import static java.lang.Math.abs;

import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;



public class FlyWheel implements Subsystem {

    public static final FlyWheel INSTANCE = new FlyWheel();
    PanelsTelemetry telemetryManager = PanelsTelemetry.INSTANCE;
    GraphManager manager = PanelsGraph.INSTANCE.getManager();
    private FlyWheel() {
    }

    public MotorEx topFW = new MotorEx("topFW");
    public MotorEx bottomFW = new MotorEx("bottomFW");

    public double goal = 1100;
    public boolean isStarted = false;
    public ControlSystem FlyWheelControl = ControlSystem.builder()
            .velPid(.009,0, 0.005)
            .elevatorFF(0.03)
            .build();


    public final Command off = new RunToVelocity(FlyWheelControl, 0.0).requires(this).named("FlywheelOff");
    public final Command on = new RunToVelocity(FlyWheelControl, 1150).requires(this).named("FlywheelOn");

    public void setGoal(double goal){
            Command on = new RunToVelocity(FlyWheelControl, goal).requires(this).named("FlywheelOn");
            on.schedule();
    }
    public final Command onAuto = new RunToVelocity(FlyWheelControl, 1550).requires(this).named("FlywheelOn");


    @Override
    public void initialize() {
        FlyWheelControl.setGoal(new KineticState(FlyWheel.INSTANCE.topFW.getCurrentPosition(), 0));
    }

    @Override
    public void periodic() {

        telemetryManager.getTelemetry().addData("left state", topFW.getState().toString());
        telemetryManager.getTelemetry().addData("right state", bottomFW.getState().toString());
        telemetryManager.getTelemetry().addData("goal: ",goal);

        telemetryManager.getTelemetry().update();
        manager.update();

        if(isStarted) {
            topFW.setPower(FlyWheelControl.calculate(topFW.getState()));
            bottomFW.setPower(FlyWheelControl.calculate(new KineticState(bottomFW.getCurrentPosition(), abs(bottomFW.getVelocity()))));
        }
    }
}