package org.firstinspires.ftc.teamcode;


import static dev.nextftc.bindings.Bindings.button;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subSystems.Spindexer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Distance;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.ServoEx;

@TeleOp(name = "Test OpMode")

public class TestTeleop extends NextFTCOpMode {


    DriverControlledCommand driverControlled = new PedroDriverControlled(
            Gamepads.gamepad1().leftStickY().negate(),
            Gamepads.gamepad1().leftStickX().negate(),
            Gamepads.gamepad1().rightStickX().negate(),
            false
    );


    public TestTeleop() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public static Limelight3A limelight = null;

    @Override
    public void onInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        PedroComponent.follower().setStartingPose(new Pose(72,72, Math.toRadians(180)));

    }

    @Override
    public void onStartButtonPressed() {
        driverControlled.schedule();
    }

    private Pose getRobotPoseFromCamera(Pose2D pose, DistanceUnit d) {
        return new Pose(pose.getX(d) + 72, pose.getY(d) + 72, pose.getHeading(AngleUnit.RADIANS), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
    private Pose getFTCPoseAsPedro(Pose2D ftcPose){
            return new Pose(ftcPose.getY(DistanceUnit.INCH) + 72, Math.abs(ftcPose.getX(DistanceUnit.INCH) - 72));
    }

    public Pose2D botpose2D;
    public Pose botposeAsPedro;
    @Override
    public void onUpdate() {


        PedroComponent.follower().update();

        LLResult result = limelight.getLatestResult();

        telemetry.addData("PedroLocalizer", PedroComponent.follower().getPose());
        if(result != null){
            if(result.isValid()){
                botpose2D = new Pose2D(DistanceUnit.INCH, result.getBotpose().getPosition().x * 39.37008 , result.getBotpose().getPosition().y * 39.37008, AngleUnit.RADIANS, result.getBotpose().getOrientation().getYaw());
                botposeAsPedro = getFTCPoseAsPedro(botpose2D);
                telemetry.addData("Limelight Coordinates As Pedro: ", getFTCPoseAsPedro(botpose2D));
                telemetry.addData("Limelight Coordinates: ", botpose2D);

                PedroComponent.follower().setPose(new Pose(botposeAsPedro.getX(), botposeAsPedro.getY(), PedroComponent.follower().getHeading()));
            }
        }
        telemetry.update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
