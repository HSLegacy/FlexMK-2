package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static dev.nextftc.bindings.Bindings.button;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subSystems.Spindexer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

import java.util.List;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@TeleOp(name = "NewTeleOp")

public class NewTeleop extends NextFTCOpMode {

    CRServoEx intake = new CRServoEx("intake");
    CRServoEx rUptake = new CRServoEx("rUptake");
    CRServoEx lUptake = new CRServoEx("lUptake");

    LLResultTypes.FiducialResult lastResult = null;

    boolean running = true;
    public int x = 0;
    public int y = 0;
    public boolean z = false;
    private boolean toggleLock = false;

    Button intakePosButton = button(() -> gamepad1.dpad_left);
    Button outakePosButton = button(() -> gamepad1.dpad_right);


    DriverControlledCommand driverControlled = new PedroDriverControlled(
            Gamepads.gamepad1().leftStickY().negate(),
            Gamepads.gamepad1().leftStickX().negate(),
            Gamepads.gamepad1().rightStickX().negate(),
            false
    );


    public NewTeleop() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Spindexer.INSTANCE),
                new SubsystemComponent(FlyWheel.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }


    @Override
    public void onStartButtonPressed() {
        driverControlled.schedule();

        intakePosButton.whenBecomesTrue(() -> switchIntakePos());
        outakePosButton.whenBecomesTrue(() -> switchOuttakePos());
        button(() -> gamepad1.b)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> runFlyWheel())
                .whenBecomesFalse(() -> stopFlyWheel());

        button(() -> gamepad1.left_bumper)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> intake.setPower(-1))
                .whenBecomesFalse(() -> intake.setPower(0));

        button(() -> gamepad1.right_bumper)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Uptake())
                .whenBecomesFalse(() -> UptakeOff());
        button(() -> gamepad1.y)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> toggleLock = true)
                .whenBecomesFalse(() -> toggleLock = true);
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        telemetry.addData("spindexer Pos", Spindexer.INSTANCE.spindexer.getState().toString());
        telemetry.addData("spindexer Foal", Spindexer.INSTANCE.spindexerControl.getGoal().getPosition());
        telemetry.addData("x number", x);
        telemetry.update();

        if (Spindexer.INSTANCE.spindexer.getCurrentPosition() > Spindexer.INSTANCE.spindexerControl.getGoal().getPosition() + 10) {
            rUptake.setPower(1);
            lUptake.setPower(1);
            z = false;
        } else if (Spindexer.INSTANCE.spindexer.getCurrentPosition() < Spindexer.INSTANCE.spindexerControl.getGoal().getPosition() - 10) {
            rUptake.setPower(-1);
            lUptake.setPower(-1);
            z = false;
        } else {
            if (z == false) {
                rUptake.setPower(0);
                lUptake.setPower(0);
            }
        }
    }

    public static Limelight3A limelight = null;

    @Override
    public void onInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }

    Runnable runFlyWheel(){
        FlyWheel.INSTANCE.on.schedule();
        return null;
    }
    Runnable stopFlyWheel(){
        FlyWheel.INSTANCE.off.schedule();
        return null;
    }
    Runnable Uptake(){
        lUptake.setPower(-1);
        rUptake.setPower(1);
        z = true;
        return null;
    }
    Runnable UptakeOff(){
        lUptake.setPower(0);
        rUptake.setPower(0);
        z = false;
        return null;
    }

    Runnable switchIntakePos(){
        if (x == 0){
            Spindexer.INSTANCE.intakePos1.schedule();
            x = 1;
        } else if (x == 1){
            Spindexer.INSTANCE.intakePos2.schedule();
            x = 2;
        } else if (x ==2){
            Spindexer.INSTANCE.intakePos3.schedule();
            x = 0;
        }
        return null;
    }

    Runnable switchOuttakePos(){
        if (y == 0){
            Spindexer.INSTANCE.outakePos1.schedule();
            y = 1;
        } else if (y == 1){
            Spindexer.INSTANCE.outakePos2.schedule();
            y = 2;
        } else if (y ==2){
            Spindexer.INSTANCE.outakePos3.schedule();
            y = 0;
        }
        return null;
    }
}
