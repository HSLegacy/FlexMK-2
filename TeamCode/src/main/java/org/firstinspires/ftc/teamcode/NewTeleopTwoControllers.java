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

@TeleOp(name = "NewTeleOpTwoControllers")

public class NewTeleopTwoControllers extends NextFTCOpMode {

    CRServoEx intake = new CRServoEx("intake");
    CRServoEx rUptake = new CRServoEx("rUptake");
    CRServoEx lUptake = new CRServoEx("lUptake");

    LLResultTypes.FiducialResult lastResult = null;
    private DigitalChannel limitSwitch = null;

    boolean running = true;
    double currentPose = 0;
    public int x = 0;
    public int y = 0;
    public double flyWheelGoal = 0;
    public boolean z = false;
    private boolean toggleLock = false;

    Button intakePosButton = button(() -> gamepad2.dpad_left);
    Button outakePosButton = button(() -> gamepad2.dpad_right);
    Button shootMacro = button(() -> gamepad2.left_bumper);
    Button offsetSpin = button(() -> gamepad2.x);
    Button offsetReset = button(() -> gamepad2.a);

    Button  powerUpSmall = button(() -> gamepad1.dpad_up);
    Button  powerDownSmall = button(() -> gamepad1.dpad_down);
    Button  powerUpBig = button(() -> gamepad1.dpad_right);
    Button  powerDownBig = button(() -> gamepad1.dpad_left);


    DriverControlledCommand driverControlled = new PedroDriverControlled(
            Gamepads.gamepad1().leftStickY().negate(),
            Gamepads.gamepad1().leftStickX().negate(),
            Gamepads.gamepad1().rightStickX().negate(),
            false
    );


    public NewTeleopTwoControllers() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(FlyWheel.INSTANCE),
                new SubsystemComponent(Spindexer.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }


    @Override
    public void onStartButtonPressed() {
        driverControlled.schedule();

        intakePosButton.whenBecomesTrue(() -> switchIntakePos());
        offsetSpin.whenBecomesTrue(() -> offset());
        offsetReset.whenBecomesTrue(() -> twooffset());
        shootMacro.whenBecomesTrue(() -> shoot3().schedule());
        outakePosButton.whenBecomesTrue(() -> switchOuttakePos());
        powerUpSmall.whenBecomesTrue(() -> flyWheelGoal += 10);
        powerDownSmall.whenBecomesTrue(() -> flyWheelGoal -= 10);
        powerUpBig.whenBecomesTrue(() -> flyWheelGoal += 100);
        powerDownBig.whenBecomesTrue(() -> flyWheelGoal -= 100);
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

        button(() -> gamepad2.right_bumper)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Uptake())
                .whenBecomesFalse(() -> UptakeOff());

        button(() -> gamepad1.y)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Turret.INSTANCE.lockOn())
                .whenBecomesFalse(() -> Turret.INSTANCE.lockOff());
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        telemetry.addData("spindexer Pos", Spindexer.INSTANCE.spindexer.getState().toString());
        telemetry.addData("spindexer Foal", Spindexer.INSTANCE.spindexerControl.getGoal().getPosition());
        telemetry.addData("spindexer encoder", Spindexer.INSTANCE.spindexer.getRawTicks());
        telemetry.addData("Flywheel Goal", flyWheelGoal);
        telemetry.addData("x number", x);
        telemetry.update();

        if (Spindexer.INSTANCE.spindexer.getCurrentPosition() > Spindexer.INSTANCE.spindexerControl.getGoal().getPosition() + 15) {
            rUptake.setPower(1);
            lUptake.setPower(1);
            z = false;
        } else if (Spindexer.INSTANCE.spindexer.getCurrentPosition() < Spindexer.INSTANCE.spindexerControl.getGoal().getPosition() - 15) {
            rUptake.setPower(-1);
            lUptake.setPower(-1);
            z = false;
        } else {
            if (z == false) {
                rUptake.setPower(0);
                lUptake.setPower(0);
            }
        }
        //Turret.INSTANCE.lockOn(limelight);

        if(!limitSwitch.getState())
            Spindexer.INSTANCE.spindexer.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FlyWheel.INSTANCE.setGoal(flyWheelGoal);
        Turret.INSTANCE.lockOnUpdate(limelight, telemetry);

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
        FlyWheel.INSTANCE.on.schedule();
        return null;
    }
    Runnable stopFlyWheel(){
        FlyWheel.INSTANCE.off.schedule();
        return null;
    }


    private Command offset() {
        Spindexer.INSTANCE.intakePos1.schedule();
        Spindexer.INSTANCE.spindexer.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        return null;

    }
    private Command twooffset() {
        Spindexer.INSTANCE.telopOffset.schedule();

        return null;

    }

    private SequentialGroup shoot3() {
        return new SequentialGroup(
                Spindexer.INSTANCE.outakePos1,
                new Delay(.5),
                runUptake,
                new Delay(2),
                Spindexer.INSTANCE.outakePos2,
                new Delay(.5),
                runUptake,
                new Delay(2),
                Spindexer.INSTANCE.outakePos3,
                new Delay(.5),
                runUptake
        );


    }

    Runnable offset2(){
        Spindexer.INSTANCE.spindexer.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Spindexer.INSTANCE.spindexerControl.setLastMeasurement(new KineticState(currentPose, Spindexer.INSTANCE.spindexerControl.getLastMeasurement().getVelocity(), Spindexer.INSTANCE.spindexerControl.getLastMeasurement().getAcceleration()));
        return null;
    }

    public ParallelGroup runUptake = new ParallelGroup(new SetPower(lUptake, -1), new SetPower(rUptake, 1));
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
