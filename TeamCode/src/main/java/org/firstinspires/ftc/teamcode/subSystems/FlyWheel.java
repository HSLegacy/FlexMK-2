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

    public MotorEx leftFlyWheel = new MotorEx("lfw");
    public MotorEx rightFlyWheel = new MotorEx("rfw")
            .reversed();

    public double goal = 1100;
    private ControlSystem FlyWheelControl = ControlSystem.builder()
            .velPid(FlyWheelConstants.P,FlyWheelConstants.I,FlyWheelConstants.D) //.008 0 0.002
            .elevatorFF(0.03)
            .build();

    /*private ControlSystem leftFlyWheelControl = ControlSystem.builder()
            .velPid(0.008,0.0,0.001) //.008 0 0.002
            .elevatorFF(0.03)
            .build();

    private ControlSystem rightFlyWheelControl = ControlSystem.builder()
            .velPid(0.008,0.0,0.001) //.008 0 0.002
            .elevatorFF(0.03)
            .build();


    public final Command off = new RunToVelocity(leftFlyWheelControl, 0.0).requires(this).named("FlywheelOff");
    public final Command on = new RunToVelocity(leftFlyWheelControl, goal).requires(this).named("FlywheelOff");

    public final Command off1 = new RunToVelocity(rightFlyWheelControl, 0.0).requires(this).named("FlywheelOff");
    public final Command on1 = new RunToVelocity(rightFlyWheelControl, -goal).requires(this).named("FlywheelOff");

     */
    public final Command off = new RunToVelocity(FlyWheelControl, 0.0).requires(this).named("FlywheelOff");
    public final Command on = new RunToVelocity(FlyWheelControl, goal).requires(this).named("FlywheelOff");

    public final Command autoOff = new RunToVelocity(FlyWheelControl, 0.0).requires(this).named("FlywheelOff");
    public final Command autoOn = new RunToVelocity(FlyWheelControl, 1100).requires(this).named("FlywheelOff");
    private boolean isStarted = false;

    @Override
    public void initialize() {

    }

    @Override
    public void periodic() {

        telemetryManager.getTelemetry().addData("left state", leftFlyWheel.getState().toString());
        telemetryManager.getTelemetry().addData("right state", rightFlyWheel.getState().toString());

        telemetryManager.getTelemetry().update();
        manager.update();

        leftFlyWheel.setPower(FlyWheelControl.calculate(leftFlyWheel.getState()));
        rightFlyWheel.setPower(FlyWheelControl.calculate(new KineticState(rightFlyWheel.getCurrentPosition(), abs(rightFlyWheel.getVelocity()))));
        //Henry please dont touch me because of this

    }

    public void launchOn(double n){
        Command on2 = new RunToVelocity(FlyWheelControl, n).requires(this).named("FlywheelOff");
        Command on3 = new RunToVelocity(FlyWheelControl, n).requires(this).named("FlywheelOff");

        on2.schedule();
        on3.schedule();
    }
    public void launchOff(){
        new RunToVelocity(FlyWheelControl, 0).requires(this).named("FlywheelOff");
        new RunToVelocity(FlyWheelControl, 0).requires(this).named("FlywheelOff");
    }


}