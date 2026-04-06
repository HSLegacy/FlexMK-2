package org.firstinspires.ftc.teamcode.subSystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;

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

import java.util.List;

public class Turret implements Subsystem {
    public boolean opModeIsStarted = false;
    public double encoderClicksPerDeg = 360d / 1800d; //limits: -1197, 1197
    public double degsPerClick = 1800d / 360d;

    private static Turret single_instance = null;

    public Limelight3A limelight;

    public Telemetry telemetry;
    public boolean relocalizeToggle;
    public static final Turret INSTANCE = new Turret();

    private Turret() {}



    public MotorEx turretMotor = new MotorEx("turret");
    public ServoEx hood = new ServoEx("hood");

    public double flyWheelGoal;
    double xOffsetBlue = 0;
    double yOffsetBlue = 0;
    double xOffsetRed = 0;
    double yOffsetRed = 0;
    public Pose targetPoseBlue = new Pose(5, 142);
    public Pose targetPoseRed = new Pose(142, 138);
    public double distanceOffsetBlue = 0;
    public double distanceOffsetRed = 0;
    public static boolean isStarted = false;
    public boolean lockToggle;


    public ControlSystem turretControl = ControlSystem.builder()
            .posPid(0.01, 0.0, 0)
            .elevatorFF(0)
            .build();

    private double convertTo360Coordinates(double angleInDegrees){
        if(angleInDegrees < 0){
            return angleInDegrees + 360;
        }else{
            return angleInDegrees;
        }
    }

    public double degreesToTurnCorrected = 0;
    public void turretMovement(boolean isRed){

        Pose targetPosition;

        double turretRobotCoordinates = convertTo360Coordinates(turretMotor.getCurrentPosition() * encoderClicksPerDeg);

        telemetry.addData("Robot Polar Coordinates: ", turretRobotCoordinates);

        double turretPolarCoordinates;

        turretPolarCoordinates =  (turretRobotCoordinates + convertTo360Coordinates(Math.toDegrees(PedroComponent.follower().getPose().getHeading()))) % 360;

        telemetry.addData("degree conversion: ", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));

        if (isRed) {
            targetPosition = targetPoseRed;
        } else {
            targetPosition = targetPoseBlue;
        }

        double polarCoordinateTargetToRobot =  Math.toDegrees(Math.atan2(targetPosition.getY() - PedroComponent.follower().getPose().getY(), targetPosition.getX() - PedroComponent.follower().getPose().getX()));
        double degreesToTurnRaw = polarCoordinateTargetToRobot - turretPolarCoordinates;


        if (degreesToTurnRaw + 360 < Math.abs(degreesToTurnRaw)){
            degreesToTurnCorrected = degreesToTurnRaw + 360;
        }else{
            degreesToTurnCorrected = degreesToTurnRaw;
        }

        telemetry.addData("polar Corrdinate Target to robot: ", polarCoordinateTargetToRobot);
        telemetry.addData("Deg To Turn Raw: ", degreesToTurnRaw);
        telemetry.addData("Deg To Turn Corrected: ", degreesToTurnCorrected);
        telemetry.addData("turret goal: ", turretControl.getGoal().getPosition());

        telemetry.addData("Turret Polar: ", turretPolarCoordinates);
       // turretControl.setGoal(new KineticState(encoderTarget));

    }
    private Pose getFTCPoseAsPedro(Pose2D ftcPose) {
        return new Pose(ftcPose.getY(DistanceUnit.INCH) + 72, Math.abs(ftcPose.getX(DistanceUnit.INCH) - 72), PedroComponent.follower().getHeading());
    }

    private Pose botCameraPose;

    public void autoFlyWheelRegressionBlue(Telemetry telemetry) {

            //Close launch zone regression
            if (distanceOffsetBlue < 28) {
                hood.setPosition(0);
                flyWheelGoal = 5.457 * distanceOffsetBlue + 948.3328;
            } else if (distanceOffsetBlue > 28 && distanceOffsetBlue < 46) {
                hood.setPosition(.2);
                flyWheelGoal = 6.60764 * distanceOffsetBlue + 863.9963;
            } else if (distanceOffsetBlue > 46 && distanceOffsetBlue < 58) {
                hood.setPosition(.4);
                flyWheelGoal = 5.04371 * distanceOffsetBlue + 907.03093;
            } else if (distanceOffsetBlue > 58 && distanceOffsetBlue < 70) {
                hood.setPosition(.5);
                flyWheelGoal = 8.54336 * distanceOffsetBlue + 703.40026;
            } else if (distanceOffsetBlue > 70 && distanceOffsetBlue < 90) {
                hood.setPosition(.65);
                flyWheelGoal = 6.77966 * distanceOffsetBlue +820.20339;
            } else if (distanceOffsetBlue > 90 && distanceOffsetBlue < 100) {
                hood.setPosition(.85);
                flyWheelGoal = 4.56621 * distanceOffsetBlue + 1038.12785;
            } else if (distanceOffsetBlue > 100 && distanceOffsetBlue < 119) {
                hood.setPosition(.9);
                flyWheelGoal = 6.66667 * distanceOffsetBlue + 816.66667;
            } else if (distanceOffsetBlue > 119) {
                hood.setPosition(1);
                flyWheelGoal = 6.06347 * distanceOffsetBlue + 858.51434;
            }

            telemetry.addData("Function y: ", flyWheelGoal);
    }

    public void autoFlyWheelRegressionRed(Telemetry telemetry) {

            //Close launch zone regression
            if (distanceOffsetRed < 28) {
                hood.setPosition(0);
                flyWheelGoal = 5.457 * distanceOffsetRed + 948.3328;
            } else if (distanceOffsetRed > 28 && distanceOffsetRed < 46) {
                hood.setPosition(.2);
                flyWheelGoal = 6.60764 * distanceOffsetRed + 863.9963;
            } else if (distanceOffsetRed > 46 && distanceOffsetRed < 58) {
                hood.setPosition(.4);
                flyWheelGoal = 5.04371 * distanceOffsetRed + 907.03093;
            } else if (distanceOffsetRed > 58 && distanceOffsetRed < 70) {
                hood.setPosition(.5);
                flyWheelGoal = 8.54336 * distanceOffsetRed + 703.40026;
            } else if (distanceOffsetRed > 70 && distanceOffsetRed < 90) {
                hood.setPosition(.65);
                flyWheelGoal = 6.77966 * distanceOffsetRed +820.20339;
            }  else if (distanceOffsetRed > 90 && distanceOffsetRed < 100) {
                hood.setPosition(.85);
                flyWheelGoal = 4.56621 * distanceOffsetRed + 1038.12785;
            } else if (distanceOffsetRed > 100 && distanceOffsetRed < 119) {
                hood.setPosition(.9);
                flyWheelGoal = 6.66667 * distanceOffsetRed + 816.66667;
            } else if (distanceOffsetRed > 119) {
                hood.setPosition(1);
                flyWheelGoal = 6.06347 * distanceOffsetRed + 858.51434;
            }

            telemetry.addData("Function y: ", flyWheelGoal);
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
        opModeIsStarted = false;
    }

    @Override
    public void periodic () {

        xOffsetBlue = PedroComponent.follower().getPose().getX() - targetPoseBlue.getX();
        yOffsetBlue = PedroComponent.follower().getPose().getY() - targetPoseBlue.getY();
        distanceOffsetBlue = Math.sqrt(Math.pow(xOffsetBlue, 2) + Math.pow(yOffsetBlue, 2));

        xOffsetRed = PedroComponent.follower().getPose().getX() - targetPoseRed.getX(); //red regresion position
        yOffsetRed = PedroComponent.follower().getPose().getY() - targetPoseRed.getY(); //red regression position
        distanceOffsetRed = Math.sqrt(Math.pow(xOffsetRed, 2) + Math.pow(yOffsetRed, 2));

        if (opModeIsStarted) {
            turretMotor.setPower(turretControl.calculate(turretMotor.getState()));
            telemetry.addData("turret Clicks: ", turretMotor.getCurrentPosition());

            if ((turretControl.getGoal().getPosition() + (degreesToTurnCorrected * degsPerClick) > -652 && (turretControl.getGoal().getPosition() + (degreesToTurnCorrected * degsPerClick) < 652)))
            {
                turretControl.setGoal(new KineticState(turretMotor.getCurrentPosition() + (degreesToTurnCorrected * degsPerClick)));
            }

        }

    }

}