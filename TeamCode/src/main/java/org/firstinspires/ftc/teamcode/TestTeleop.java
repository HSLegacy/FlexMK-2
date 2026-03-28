package org.firstinspires.ftc.teamcode;


import static dev.nextftc.bindings.Bindings.button;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "Test Auto")

public class TestTeleop extends NextFTCOpMode {


    public MotorEx turretMotor = new MotorEx("turret");
    public GoBildaPinpointDriver pinpoint;
    DriverControlledCommand driverControlled = new PedroDriverControlled(
            Gamepads.gamepad1().leftStickY().negate(),
            Gamepads.gamepad1().leftStickX().negate(),
            Gamepads.gamepad1().rightStickX().negate(),
            false
    );
    public static Limelight3A limelight = null;

    Turret turret = Turret.getInstance(limelight, telemetry);
    public TestTeleop() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(turret)
        );
    }



    @Override
    public void onInit() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        //PedroComponent.follower().setStartingPose(new Pose(72,72, Math.toRadians(90)));

    }

    @Override
    public void onStartButtonPressed() {
        turretMotor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PedroComponent.follower().setPose(new Pose(0,0,0));
            driverControlled.schedule();
        turret.opModeIsStarted = true;
    }

    private Pose getRobotPoseFromCamera(Pose2D pose, DistanceUnit d) {
        return new Pose(pose.getX(d) + 72, pose.getY(d) + 72, pose.getHeading(AngleUnit.RADIANS), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    private Pose getFTCPoseAsPedro(Pose2D ftcPose) {
        return new Pose(ftcPose.getY(DistanceUnit.INCH) + 72, Math.abs(ftcPose.getX(DistanceUnit.INCH) - 72), PedroComponent.follower().getHeading());
    }

    public Pose2D botpose2D;
    public Pose botposeAsPedro;
    public boolean localizerSwitch = false;
    public boolean cameraResultValid = false;
    public Pose botCameraPose = new Pose(0, 0, 0);

    public Pose targetPoseBlue = new Pose(7, 137);

    public double degreesToTurnFromZero;

    public double turretHeadingOffset;

    public double encoderClicksPerDeg = 360d / 5081d; //limits: -1197, 1197
    public double degreesToTurnFromTurret;
    public double turretPolarCoordinates;

    @Override
    public void onUpdate() {

        if (gamepad1.a){
            resetButton();
        }
        if (gamepad1.b){
            turret.turretControl.setGoal(new KineticState(-300));
        }
        else if(gamepad1.y){
            turret.turretControl.setGoal(new KineticState(0));
        }

        LLResult result = limelight.getLatestResult();


        if (result != null) {
            if (result.isValid()) {

                cameraResultValid = true;

                botpose2D = new Pose2D(DistanceUnit.INCH, result.getBotpose().getPosition().x * 39.37008, result.getBotpose().getPosition().y * 39.37008, AngleUnit.RADIANS, PedroComponent.follower().getHeading());
                botposeAsPedro = getFTCPoseAsPedro(botpose2D);


                degreesToTurnFromZero = Math.toDegrees(Math.atan2(targetPoseBlue.getX() - botposeAsPedro.getX(), targetPoseBlue.getY() - botposeAsPedro.getY())) - Math.toDegrees(PedroComponent.follower().getHeading());


                localizerSwitch = true;

                botCameraPose = new Pose(botposeAsPedro.getX(), botposeAsPedro.getY(), PedroComponent.follower().getHeading());
            } else {
                cameraResultValid = false;
            }

        }

        telemetry.update();
        PedroComponent.follower().update();

        //turretMotor.setPower(turretControl.calculate(turretMotor.getState()));
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }

    Runnable resetButton() {
        if (cameraResultValid) {
            PedroComponent.follower().setPose(botCameraPose);
        }
        return null;
    }
}
