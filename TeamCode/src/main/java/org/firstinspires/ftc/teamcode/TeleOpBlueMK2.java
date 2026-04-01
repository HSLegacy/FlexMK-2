package org.firstinspires.ftc.teamcode;

import static dev.nextftc.bindings.Bindings.button;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.control.KineticState;
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

@TeleOp(name = "TeleOpBlueMK2")

public class TeleOpBlueMK2 extends NextFTCOpMode {

    MotorEx intake = new MotorEx("intake");
    CRServoEx uptake = new CRServoEx("uptake");
    ServoEx gate = new ServoEx("door");
    ServoEx hood = new ServoEx("hood");

    Button relocalize = button(() -> gamepad1.x);
    Button resetHeading = button(() -> gamepad1.y);
    DriverControlledCommand driverControlled = new PedroDriverControlled(
            Gamepads.gamepad1().leftStickY(),
            Gamepads.gamepad1().leftStickX(),
            Gamepads.gamepad1().rightStickX().negate(),
            false
    );

    Turret turret = Turret.getInstance(limelight, telemetry);
    public TeleOpBlueMK2() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(FlyWheel.INSTANCE),
                new SubsystemComponent(turret),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    ElapsedTime timer;

    @Override
    public void onStartButtonPressed() {
        gate.setPosition(0.5);
        FlyWheel.INSTANCE.isStarted = true;
        driverControlled.schedule();
        turret.opModeIsStarted = true;

        timer = new ElapsedTime();

        relocalize.whenBecomesTrue(() -> turret.resetButton());
        resetHeading.whenBecomesTrue(() -> PedroComponent.follower().setPose(new Pose(0, 0, Math.toRadians(180))));
        button(() -> gamepad1.a)
                .whenBecomesTrue(() -> shootTimer());
        button(() -> gamepad1.b)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> runFlyWheel())
                .whenBecomesFalse(() -> stopFlyWheel());
        button(() -> gamepad1.left_bumper)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> intake.setPower(-1))
                .whenBecomesTrue(() -> uptake.setPower(1))
                .whenBecomesFalse(() -> intake.setPower(0))
                .whenBecomesFalse(() -> uptake.setPower(0));
        button(() -> gamepad1.right_bumper)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> intake.setPower(1))
                .whenBecomesTrue(() -> uptake.setPower(-1))
                .whenBecomesFalse(() -> intake.setPower(0))
                .whenBecomesFalse(() -> uptake.setPower(0));
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        telemetry.update();

        turret.relocalizationUpdate(limelight, telemetry);
        turret.autoFlyWheelRegressionBlue(telemetry);
        turret.turretMovement(false);

        FlyWheel.INSTANCE.FlyWheelControl.setGoal(new KineticState(0, turret.flyWheelGoal));

        telemetry.addData("localizper:", PedroComponent.follower().getPose());

        if((timer.seconds() - timeWhenShot) > 3.0){
            gate.setPosition(0.5);
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

    Runnable runFlyWheel() {
        FlyWheel.INSTANCE.on.schedule();
        return null;
    }

    Runnable stopFlyWheel() {
        FlyWheel.INSTANCE.off.schedule();
        return null;
    }
    double timeWhenShot;
    Runnable shootTimer(){
        gate.setPosition(1);
        timeWhenShot = timer.seconds();
        return null;
    }

}