package org.firstinspires.ftc.teamcode.subSystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utilities.MathUtils;

import java.util.List;

public class Turret implements Subsystem {
    public boolean opModeIsStarted = false;
    public double encoderClicksPerDeg = 5081 / 360.0; //limits: -1197, 1197

    private static Turret single_instance = null;

    private static Limelight3A limelight;

    private static Telemetry telemetry;
    public boolean relocalizeToggle;

    private Turret() {}

    public static synchronized Turret getInstance(Limelight3A l, Telemetry tel)
    {
        if (single_instance == null)
            single_instance = new Turret();

        limelight = l;
        telemetry = tel;
        return single_instance;
    }

    public MotorEx turretMotor = new MotorEx("turret");
    public ServoEx hood = new ServoEx("hood");

    public double flyWheelGoal;

    public static boolean isStarted = false;
    public double lastHeading = 0;
    public boolean lockToggle;


    public ControlSystem turretControl = ControlSystem.builder()
            .posPid(0.008, 0.0, 0.0001)
            .elevatorFF(0)
            .build();


    private void checkForValidTag(){
        LLResult result = limelight.getLatestResult();

        telemetry.addData("Pedro Localizer", PedroComponent.follower().getPose());

        if (result != null) {
            if (result.isValid()) {
                turretMovement(result);
            }
        }
    }

    private void turretMovement(LLResult result){
        List<LLResultTypes.FiducialResult> feducialResults = result.getFiducialResults();

        LLResultTypes.FiducialResult lastResult = feducialResults.get(0);
        Pose targetPosition;
        Pose targetPoseBlue = new Pose(7, 137);
        Pose targetPoseRed = new Pose(7, 137);

        double turretPolarCoordinates = turretMotor.getCurrentPosition() * encoderClicksPerDeg + Math.toDegrees(PedroComponent.follower().getPose().getHeading());

        switch(lastResult.getFiducialId()){
            case 22:
                targetPosition = targetPoseBlue;
                break;
            case 21:
                targetPosition = targetPoseRed;
                break;
            default:
                return; // don't run anymore code if the motif is detected
        }

        double polarCoordinateTarget = turretPolarCoordinates + Math.atan2(targetPosition.getY(), targetPosition.getX());

        double encoderTarget = MathUtils.clamp(polarCoordinateTarget, -1197, 1197);
        telemetry.addData("encoderTarget: ", encoderTarget);
        telemetry.update();
       // turretControl.setGoal(new KineticState(encoderTarget));

    }
    public void lockOnUpdate() {

        checkForValidTag();
        //telemetry.update();
    }
    private Pose getFTCPoseAsPedro(Pose2D ftcPose) {
        return new Pose(ftcPose.getY(DistanceUnit.INCH) + 72, Math.abs(ftcPose.getX(DistanceUnit.INCH) - 72), PedroComponent.follower().getHeading());
    }

    private Pose botCameraPose;
    public void relocalizationUpdate(){
        LLResult result = limelight.getLatestResult();
        Pose2D botpose2D;
        Pose botPoseAsPedro;


        if (result != null) {
            if (result.isValid()) {

                relocalizeToggle = true;

                botpose2D = new Pose2D(DistanceUnit.INCH, result.getBotpose().getPosition().x * 39.37008, result.getBotpose().getPosition().y * 39.37008, AngleUnit.RADIANS, PedroComponent.follower().getHeading());
                botPoseAsPedro = getFTCPoseAsPedro(botpose2D);


                relocalizeToggle = true;
                telemetry.addData("Limelight Coordinates As Pedro: ", getFTCPoseAsPedro(botpose2D));

                botCameraPose = new Pose(botPoseAsPedro.getX(), botPoseAsPedro.getY(), PedroComponent.follower().getHeading());
            } else {
                relocalizeToggle = false;
            }

        }
    }
    Runnable resetButton() {
        if (relocalizeToggle) {
            PedroComponent.follower().setPose(botCameraPose);
        }
        return null;
    }
//hi

    @Override
    public void initialize () {
    }

    @Override
    public void periodic () {
        if (opModeIsStarted) {
            //turretMotor.setPower(turretControl.calculate(turretMotor.getState()));
            lockOnUpdate();
        }
    }
}