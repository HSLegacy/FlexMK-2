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
    public MotorEx rightFlyWheel = new MotorEx("rightFW").reversed();

    public double goal = 1100;
    private ControlSystem FlyWheelControl = ControlSystem.builder()
            .velPid(.01,0,0 )
            .elevatorFF(0.03)
            .build();


    public final Command off = new RunToVelocity(FlyWheelControl, 0.0).requires(this).named("FlywheelOff");
    public final Command on = new RunToVelocity(FlyWheelControl, 1000).requires(this).named("FlywheelOn");

    private boolean isStarted = false;

    @Override
    public void initialize() {

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