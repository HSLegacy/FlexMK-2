package org.firstinspires.ftc.teamcode;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subSystems.Spindexer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.powerable.SetPower;

@TeleOp(name = "NewTeleOp")

public class NewTeleop extends NextFTCOpMode {

    CRServoEx intake = new CRServoEx("intake");
    CRServoEx rUptake = new CRServoEx("rUptake");
    CRServoEx lUptake = new CRServoEx("lUptake");

    LLResultTypes.FiducialResult lastResult = null;
    private DigitalChannel limitSwitch = null;

    boolean running = true;
    private boolean toggleLock = false;


    Button fire = button(() -> gamepad1.a);
    Button resetSpindexer = button(() -> gamepad1.dpad_down);


    DriverControlledCommand driverControlled = new PedroDriverControlled(
            Gamepads.gamepad1().leftStickY().negate(),
            Gamepads.gamepad1().leftStickX().negate(),
            Gamepads.gamepad1().rightStickX().negate(),
            false
    );


    public NewTeleop() {
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
        driverControlled.schedule();

        fire.whenBecomesTrue(() -> fireFuction());
        resetSpindexer.whenBecomesTrue(() -> Spindexer.INSTANCE.intakePosition.schedule());
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
                .whenBecomesTrue(() -> intake.setPower(1))
                .whenBecomesFalse(() -> intake.setPower(0));

        button(() -> gamepad1.y)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Turret.INSTANCE.lockedOn = true)
                .whenBecomesFalse(() -> Turret.INSTANCE.lockedOn = false);
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        telemetry.addData("spindexer Pos", Spindexer.INSTANCE.spindexer.getState().toString());
        telemetry.addData("spindexer Goal", Spindexer.INSTANCE.spindexerControl.getGoal().getPosition());
        telemetry.addData("spindexer encoder", Spindexer.INSTANCE.spindexer.getRawTicks());
        telemetry.addData("Flywheel Goal", Turret.INSTANCE.flyWheelGoal);
        telemetry.update();

        if(!limitSwitch.getState() && Spindexer.INSTANCE.spindexerControl.getGoal().getPosition() != 0){
            Spindexer.INSTANCE.spindexer.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Spindexer.INSTANCE.intakePosition.schedule();
            lUptake.setPower(0);
            rUptake.setPower(0);
        }

        Turret.INSTANCE.lockOnUpdate(limelight, telemetry);
        FlyWheel.INSTANCE.setGoal(Turret.INSTANCE.flyWheelGoal);

    }

    public static Limelight3A limelight = null;

    @Override
    public void onInit() {
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

    Runnable runFlyWheel(){
        Turret.INSTANCE.flyWheelGoal = 1150;
        return null;
    }
    Runnable stopFlyWheel(){
        Turret.INSTANCE.flyWheelGoal = 0;
        return null;
    }

    Runnable fireFuction(){
        lUptake.setPower(-1);
        rUptake.setPower(1);
        Spindexer.INSTANCE.firingPosition.schedule();
        return null;
    }

}