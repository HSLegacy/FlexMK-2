package org.firstinspires.ftc.teamcode;

import static dev.nextftc.bindings.Bindings.button;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subSystems.Spindexer;
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
import dev.nextftc.hardware.impl.ServoEx;

@TeleOp(name = "NewTeleOpBlue")

public class NewTeleopBlue extends NextFTCOpMode {

    CRServoEx intake = new CRServoEx("intake");
    CRServoEx intake2 = new CRServoEx("intake2");
    CRServoEx upTakeWheel = new CRServoEx("upTakeWheel");
    ServoEx gate = new ServoEx("gate");
    ServoEx hood = new ServoEx("hood");
    Turret turret = Turret.INSTANCE;

    LLResultTypes.FiducialResult lastResult = null;
    private DigitalChannel limitSwitch = null;

    boolean running = true;
    private boolean toggleLock = false;


    Button fire = button(() -> gamepad1.a);
    Button resetSpindexer = button(() -> gamepad1.dpad_down);
    Button rightTurret = button(() -> gamepad1.dpad_right);
    Button leftTurret = button(() -> gamepad1.dpad_left);
    Button lockOn = button(() -> gamepad1.dpad_up);

    DriverControlledCommand driverControlled = new PedroDriverControlled(
            Gamepads.gamepad1().leftStickY().negate(),
            Gamepads.gamepad1().leftStickX().negate(),
            Gamepads.gamepad1().rightStickX().negate(),
            false
    );


    public NewTeleopBlue() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(FlyWheel.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE),
                new SubsystemComponent(Spindexer.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }


    @Override
    public void onStartButtonPressed() {
        Spindexer.INSTANCE.intakePosition.schedule();
        FlyWheel.INSTANCE.isStarted = true;
        Spindexer.INSTANCE.isStarted = true;
        turret.turretPower = true;
        driverControlled.schedule();
        Spindexer.INSTANCE.spindexerControl.setGoal(new KineticState(160));
        fire.whenBecomesTrue(() -> fireFuction());
        rightTurret.whenBecomesTrue(() -> rightTurret());
        leftTurret.whenBecomesTrue(() -> leftTurret());
        lockOn.whenBecomesTrue(() -> lockOn());
        resetSpindexer.whenBecomesTrue(() -> Spindexer.INSTANCE.spindexerControl.setGoal(new KineticState(160)));
        button(() -> gamepad1.x)
                .whenBecomesTrue(() -> follower().setPose(new Pose(0, 0, 0)));
        button(() -> gamepad1.b)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> runFlyWheel())
                .whenBecomesFalse(() -> stopFlyWheel());
        button(() -> gamepad1.dpad_up)
                .whenBecomesTrue(() -> hood.setPosition(.12));
        button(() -> gamepad1.left_bumper)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> intake.setPower(1))
                .whenBecomesTrue(() -> intake2.setPower(-1))
                .whenBecomesTrue(() -> gate.setPosition(.97))
                .whenBecomesTrue(() -> upTakeWheel.setPower(0))
                .whenBecomesFalse(() -> intake.setPower(-1))
                .whenBecomesFalse(() -> intake2.setPower(1))
                .whenBecomesFalse(() -> gate.setPosition(.75));

        button(() -> gamepad1.right_bumper)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> intake.setPower(-1))
                .whenBecomesTrue(() -> intake2.setPower(1))
                .whenBecomesFalse(() -> intake2.setPower(0))
                .whenBecomesFalse(() -> intake.setPower(0));
        /*
        button(() -> gamepad1.y)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Turret.INSTANCE.lockOn(limelight, telemetry))
                .whenBecomesFalse(() -> Turret.INSTANCE.lockOn(limelight, telemetry));

         */
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        telemetry.addData("spindexer Pos", Spindexer.INSTANCE.spindexer.getState().toString());
        telemetry.addData("spindexer Goal", Spindexer.INSTANCE.spindexerControl.getGoal().getPosition());
        telemetry.addData("spindexer encoder", Spindexer.INSTANCE.spindexer.getRawTicks());
        telemetry.addData("Flywheel Goal", Turret.INSTANCE.flyWheelGoal);
        telemetry.update();

        if(Spindexer.INSTANCE.spindexer.getMotor().getCurrentPosition() < -1100 && Spindexer.INSTANCE.spindexer.getMotor().getCurrentPosition() > -1120){
            Spindexer.INSTANCE.spindexerControl.setGoal(new KineticState(160));
        }

        /*
        if (!limitSwitch.getState() && Spindexer.INSTANCE.spindexerControl.getGoal().getPosition() == 0){
            Spindexer.INSTANCE.spindexer.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Spindexer.INSTANCE.spindexerControl.setGoal(new KineticState(160));
        }
        */

        FlyWheel.INSTANCE.setGoal(turret.flyWheelGoal);

        turret.lockOnUpdate(limelight, telemetry);
        turret.lockOnTurretBlue(limelight, telemetry);

    }

    public static Limelight3A limelight = null;

    @Override
    public void onInit() {
        turret.turretControl.setGoal(new KineticState(0));
        Spindexer.INSTANCE.intakePosition.schedule();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }

    Runnable runFlyWheel() {
        turret.flyWheelGoal = 1150;
        return null;
    }

    Runnable stopFlyWheel() {
        turret.flyWheelGoal = 0;
        return null;
    }

    Runnable fireFuction() {
        if (gate.getPosition() == .75) {
            upTakeWheel.setPower(1);
            gate.setPosition(.75);
            Spindexer.INSTANCE.firingPosition.schedule();
        }
        return null;
    }

    Runnable rightTurret() {
        Turret.INSTANCE.turretControl.setGoal(new KineticState(Turret.INSTANCE.turretControl.getGoal().getPosition() - 200));
        return null;
    }

    Runnable leftTurret() {
        Turret.INSTANCE.turretControl.setGoal(new KineticState(Turret.INSTANCE.turretControl.getGoal().getPosition() + 200));
        return null;
    }

    Runnable lockOn() {
        if ((((Turret.INSTANCE.lastHeading - Math.toDegrees(PedroComponent.follower().getHeading())) * Turret.INSTANCE.encoderClicksPerDeg) + Turret.INSTANCE.turretControl.getGoal().getPosition()) < 1196 && (((Turret.INSTANCE.lastHeading - Math.toDegrees(PedroComponent.follower().getHeading())) * Turret.INSTANCE.encoderClicksPerDeg) + Turret.INSTANCE.turretControl.getGoal().getPosition()) > -1196) {
            Turret.INSTANCE.turretControl.setGoal(new KineticState(((Turret.INSTANCE.lastHeading - Math.toDegrees(PedroComponent.follower().getHeading())) * Turret.INSTANCE.encoderClicksPerDeg) + Turret.INSTANCE.turretControl.getGoal().getPosition()));
        }
        return null;
    }
    Runnable turretCorrectOn() {
        Turret.INSTANCE.lockToggle = false;
        return null;
    }
    Runnable turretCorrectOff() {
        Turret.INSTANCE.lockToggle = true;
        return null;
    }
}