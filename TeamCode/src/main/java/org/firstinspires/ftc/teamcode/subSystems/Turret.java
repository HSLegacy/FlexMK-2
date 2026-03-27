package org.firstinspires.ftc.teamcode.subSystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public double encoderClicksPerDeg = 360d / 1800d; //limits: -1197, 1197

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
    double xOffset = 0;
    double yOffset = 0;
    Pose targetPoseBlue = new Pose(7, 137);
    Pose targetPoseRed = new Pose(137, 137);
    public double distanceOffset = 0;
    public static boolean isStarted = false;
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
    public void lockOnUpdate(){
        checkForValidTag();
    }

    private double convertTo360Coordinates(double angleInDegrees){
        if(angleInDegrees < 0){
            return angleInDegrees + 360;
        }else{
            return angleInDegrees;
        }
    }
    private void turretMovement(LLResult result){
        List<LLResultTypes.FiducialResult> feducialResults = result.getFiducialResults();

        LLResultTypes.FiducialResult lastResult = feducialResults.get(0);
        Pose targetPosition;

        double turretRobotCoordinates = convertTo360Coordinates(turretMotor.getCurrentPosition() * encoderClicksPerDeg);

        telemetry.addData("Robot Polar Coordinates: ", turretRobotCoordinates);

        double turretPolarCoordinates;

        turretPolarCoordinates =  turretRobotCoordinates - convertTo360Coordinates(Math.toDegrees(PedroComponent.follower().getPose().getHeading()));

        telemetry.addData("degree conversion: ", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        targetPosition = targetPoseRed;
        /*
        switch(lastResult.getFiducialId()){
            case 22:
                targetPosition = targetPoseBlue;
                break;
            case 21:
                targetPosition = targetPoseRed;
                break;
            default:
                targetPosition = targetPoseBlue;
        }
         */

        double polarCoordinateTarget = turretPolarCoordinates + Math.toDegrees(Math.atan2(targetPosition.getY(), targetPosition.getX()));

        telemetry.addData("polar Corrdinate Target: ", polarCoordinateTarget);
        telemetry.addData("turretPolarCoordinates", turretPolarCoordinates);
       // turretControl.setGoal(new KineticState(encoderTarget));

    }
    private Pose getFTCPoseAsPedro(Pose2D ftcPose) {
        return new Pose(ftcPose.getY(DistanceUnit.INCH) + 72, Math.abs(ftcPose.getX(DistanceUnit.INCH) - 72), PedroComponent.follower().getHeading());
    }

    private Pose botCameraPose;

    public void autoFlyWheelRegression(Limelight3A limelight, Telemetry telemetry) {

        LLResult result = limelight.getLatestResult();

        if (result != null) {

            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> feducialResults = result.getFiducialResults();
                LLResultTypes.FiducialResult lastResult = feducialResults.get(0);

                if (lastResult != null) {

                    //Close launch zone regression
                    if (distanceOffset < 28) {
                        hood.setPosition(0);
                        flyWheelGoal = 5.457 * distanceOffset + 948.3328;
                    } else if (distanceOffset > 28 && distanceOffset < 46) {
                        hood.setPosition(.2);
                        flyWheelGoal = 6.60764 * distanceOffset + 863.9963;
                    } else if (distanceOffset > 46 && distanceOffset < 58) {
                        hood.setPosition(.4);
                        flyWheelGoal = 5.04371 * distanceOffset + 907.03093;
                    } else if (distanceOffset > 58 && distanceOffset < 70) {
                        hood.setPosition(.5);
                        flyWheelGoal = 8.54336 * distanceOffset + 703.40026;
                    } else if (distanceOffset > 70 && distanceOffset < 90) {
                        hood.setPosition(.65);
                        flyWheelGoal = 6.77966 * distanceOffset +820.20339;
                    } else if (distanceOffset > 90) {
                        hood.setPosition(.9);
                        flyWheelGoal = 3.10128 * distanceOffset + 1262.21009;
                    }

                    telemetry.addData("Function y: ", flyWheelGoal);
                }
            }
        }
    }

    public void relocalizationUpdate(Limelight3A limelight, Telemetry telemetry){
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
     public void resetButton() {
        if (relocalizeToggle) {
            PedroComponent.follower().setPose(botCameraPose);
        }
    }

    @Override
    public void initialize () {
        turretMotor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void periodic () {

        xOffset = PedroComponent.follower().getPose().getX() - targetPoseBlue.getX();
        yOffset = PedroComponent.follower().getPose().getY() - targetPoseBlue.getY();
        distanceOffset = Math.sqrt(Math.pow(xOffset, 2) + Math.pow(yOffset, 2));

        if (opModeIsStarted) {
            //turretMotor.setPower(turretControl.calculate(turretMotor.getState()));
            telemetry.addData("turret Clicks: ", turretMotor.getCurrentPosition());
            lockOnUpdate();

        }
    }
}