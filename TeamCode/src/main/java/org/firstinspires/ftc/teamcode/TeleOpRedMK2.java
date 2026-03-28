package org.firstinspires.ftc.teamcode;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@TeleOp(name = "TeleOpRedMK2")

public class TeleOpRedMK2 extends NextFTCOpMode {

    MotorEx intake = new MotorEx("intake");
    MotorEx turretMotor = new MotorEx("turret");
    CRServoEx uptake = new CRServoEx("uptake");
    ServoEx gate = new ServoEx("door");
    ServoEx hood = new ServoEx("hood");

    Button relocalize = button(() -> gamepad1.x);
    DriverControlledCommand driverControlled = new PedroDriverControlled(
            Gamepads.gamepad1().leftStickY().negate(),
            Gamepads.gamepad1().leftStickX().negate(),
            Gamepads.gamepad1().rightStickX().negate(),
            false
    );

    Turret turret = Turret.getInstance(limelight, telemetry);
    public TeleOpRedMK2() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(FlyWheel.INSTANCE),
                new SubsystemComponent(turret),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        FlyWheel.INSTANCE.isStarted = true;
        driverControlled.schedule();
        turret.opModeIsStarted = true;

        relocalize.whenBecomesTrue(() -> turret.resetButton());
        button(() -> gamepad1.a)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> gate.setPosition(.5))
                .whenBecomesFalse(() -> gate.setPosition(1));
        button(() -> gamepad1.b)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> runFlyWheel())
                .whenBecomesFalse(() -> stopFlyWheel());
        button(() -> gamepad1.dpad_up)
                .whenBecomesTrue(() -> hood.setPosition(.8));
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
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BindingManager.update();
        telemetry.update();

        turret.relocalizationUpdate(limelight, telemetry);
        turret.autoFlyWheelRegressionRed(limelight, telemetry);

        FlyWheel.INSTANCE.FlyWheelControl.setGoal(new KineticState(0, turret.flyWheelGoal));

        telemetry.addData("localizper:", PedroComponent.follower().getPose());
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
}