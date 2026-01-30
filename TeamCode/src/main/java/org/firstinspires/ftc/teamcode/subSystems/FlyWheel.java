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
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;



public class FlyWheel implements Subsystem {

    public static final FlyWheel INSTANCE = new FlyWheel();
    PanelsTelemetry telemetryManager = PanelsTelemetry.INSTANCE;
    GraphManager manager = PanelsGraph.INSTANCE.getManager();
    private FlyWheel() {
    }

    public MotorEx leftFlyWheel = new MotorEx("leftFW");
    public MotorEx rightFlyWheel = new MotorEx("rightFW");

    public double goal = 1100;
    private ControlSystem FlyWheelControl = ControlSystem.builder()
            .velPid(.011,0, 0.8) //.011, 0, .8
            .elevatorFF(0.03)
            .build();


    public final Command off = new RunToVelocity(FlyWheelControl, 0.0).requires(this).named("FlywheelOff");
    public final Command on = new RunToVelocity(FlyWheelControl, 1150).requires(this).named("FlywheelOn");

    public void setGoal(double goal){
            Command on = new RunToVelocity(FlyWheelControl, goal).requires(this).named("FlywheelOn");
            on.schedule();
    }
    public final Command onAuto = new RunToVelocity(FlyWheelControl, 1550).requires(this).named("FlywheelOn");

    private boolean isStarted = false;

    @Override
    public void initialize() {
        FlyWheelControl.setGoal(new KineticState(FlyWheel.INSTANCE.leftFlyWheel.getCurrentPosition(), 0));
    }

    @Override
    public void periodic() {

        telemetryManager.getTelemetry().addData("left state", leftFlyWheel.getState().toString());
        telemetryManager.getTelemetry().addData("right state", rightFlyWheel.getState().toString());
        telemetryManager.getTelemetry().addData("goal: ",goal);

    /*    FlyWheelControl = ControlSystem.builder()
                .velPid(FlyWheelConstants.P,FlyWheelConstants.I,FlyWheelConstants.D) //.008 0 0.002
                .elevatorFF(0.03)
                .build();
    */
        telemetryManager.getTelemetry().update();
        manager.update();

        leftFlyWheel.setPower(FlyWheelControl.calculate(leftFlyWheel.getState()));
        rightFlyWheel.setPower(FlyWheelControl.calculate(new KineticState(rightFlyWheel.getCurrentPosition(), abs(rightFlyWheel.getVelocity()))));

    }
}