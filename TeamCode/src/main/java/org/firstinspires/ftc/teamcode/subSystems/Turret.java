package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;
import java.util.Map;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    public boolean lockedOn = false;
    public boolean zeroToggle = true;
    private static DigitalChannel turretLimitSwitch = null;
    private double encoderClicksPerDeg = 5081 / 360.0; //limits: -1197, 1197

    private Turret() {
    }

    public MotorEx turretMotor = new MotorEx("turretMotor");

    public double flyWheelGoal;
    public static boolean isStarted = false;
    boolean isBounded;
    public double lastHeading = 0;
    public double lastTurretPose = 0;
    LLResultTypes.FiducialResult lastResult = null;

    KineticState turretState = new KineticState();

    public ControlSystem turretControl = ControlSystem.builder()
            .posPid(0.008, 0.0, 0.0001)
            .elevatorFF(0)
            .build();


    public final Command goal0Turret = new RunToPosition(turretControl, 0).requires(this).named("goal0Turret");
    public final Command goal150Turret = new RunToPosition(turretControl, -800).requires(this).named("goal150Turret");

    public void lockOnUpdate(Limelight3A limelight, Telemetry telemetry) {

        LLResult result = limelight.getLatestResult();


        if (result != null) {

            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> feducialResults = result.getFiducialResults();
                //telemetry.addData("Tx:", feducialResults.get(0).getTargetXDegrees());
                lastResult = feducialResults.get(0);

                if (lastResult != null) {

                    telemetry.addData("Camera Pose Target Space: ", lastResult.getCameraPoseTargetSpace());

                    if (lastResult.getCameraPoseTargetSpace().getPosition().z > -3) {
                        flyWheelGoal = 184.4566 * Math.pow(lastResult.getCameraPoseTargetSpace().getPosition().z, 2) + 534.06469 * (lastResult.getCameraPoseTargetSpace().getPosition().z) + 1545.52227;
                    } else if (lastResult.getCameraPoseTargetSpace().getPosition().z < -3 && lastResult.getCameraPoseTargetSpace().getPosition().z > -6) {
                        flyWheelGoal = -117.70721 * lastResult.getCameraPoseTargetSpace().getPosition().z + 1157.50788;
                    }

                    telemetry.addData("Function y: ", flyWheelGoal);

                    telemetry.addData("locked on: ", lockedOn);

                    telemetry.addData("robot Yaw: ", lastResult.getTargetPoseRobotSpace().getOrientation().getYaw());

                    if ((lastResult.getTargetPoseRobotSpace().getOrientation().getYaw(AngleUnit.DEGREES) < -2 || lastResult.getTargetPoseRobotSpace().getOrientation().getYaw(AngleUnit.DEGREES) < -2) && lockedOn) {
                        new TurnBy(Angle.fromDeg(lastResult.getTargetPoseRobotSpace().getOrientation().getYaw(AngleUnit.DEGREES))).schedule();
                    }
                }
            }
        }

        //telemetry.update();

    }

    public void lockOn(Limelight3A limelight, Telemetry telemetry) {

        LLResult result = limelight.getLatestResult();


        if (result != null) {

            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> feducialResults = result.getFiducialResults();
                //telemetry.addData("Tx:", feducialResults.get(0).getTargetXDegrees());
                lastResult = feducialResults.get(0);

                if (lastResult != null) {

                    telemetry.addData("Camera Pose Target Space: ", lastResult.getCameraPoseTargetSpace());

                    telemetry.addData("Function y: ", flyWheelGoal);

                    telemetry.addData("locked on: ", lockedOn);

                    telemetry.addData("robot Yaw: ", lastResult.getTargetPoseRobotSpace().getOrientation().getYaw());

                    if ((lastResult.getTargetPoseRobotSpace().getOrientation().getYaw(AngleUnit.DEGREES) < -2 || lastResult.getTargetPoseRobotSpace().getOrientation().getYaw(AngleUnit.DEGREES) < -2) && lockedOn) {
                        new TurnBy(Angle.fromDeg(lastResult.getTargetPoseRobotSpace().getOrientation().getYaw(AngleUnit.DEGREES))).schedule();
                    }
                }
            }
        }

        //telemetry.update();

    }


    public void lockOnTurret(Limelight3A limelight, Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();


        while(isBounded){
            if(result != null){
                if(result.isValid()){
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    lastResult = fiducialResults.get(0);

                    if (lastResult.getTargetXDegrees() < -5){
                        turretControl.setGoal(new KineticState(turretControl.getGoal().getPosition() +12));
                    }
                    else if (lastResult.getTargetXDegrees() > 5){
                        turretControl.setGoal(new KineticState(turretControl.getGoal().getPosition() -12));
                    }

                    lastHeading = Math.toDegrees(PedroComponent.follower().getHeading());

                }else{
                    turretControl.setGoal(new KineticState(lastHeading - Math.toDegrees(PedroComponent.follower().getHeading())));
                }
            }

            isBounded = ((turretControl.getGoal().getPosition() < 1196 || turretControl.getGoal().getPosition() > -1196) && (lastHeading * encoderClicksPerDeg < 1196 || lastHeading * encoderClicksPerDeg > -1196));
        }
        while(!isBounded){
            turretControl.setGoal(new KineticState(0));
            isBounded = ((turretControl.getGoal().getPosition() < 1196 || turretControl.getGoal().getPosition() > -1196) && (lastHeading * encoderClicksPerDeg < 1196 || lastHeading * encoderClicksPerDeg > -1196));

            if(result != null){
                if(result.isValid()){
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    lastResult = fiducialResults.get(0);

                    lastHeading = Math.toDegrees(PedroComponent.follower().getHeading());

                }else{
                    turretControl.setGoal(new KineticState(lastHeading - Math.toDegrees(PedroComponent.follower().getHeading())));
                }
            }

        }
        /*

        if (result != null) {

            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> feducialResults = result.getFiducialResults();
                //telemetry.addData("Tx:", feducialResults.get(0).getTargetXDegrees());
                lastResult = feducialResults.get(0);

                if (lastResult != null) {

                    telemetry.addData("Camera Pose Target Space: ", lastResult.getCameraPoseTargetSpace());

                    telemetry.addData("Function y: ", flyWheelGoal);

                    telemetry.addData("locked on: ", lockedOn);

                    telemetry.addData("camera Yaw: ", lastResult.getCameraPoseTargetSpace().getOrientation().getYaw(AngleUnit.DEGREES));

                    telemetry.addData("tagetX Degree: ", lastResult.getTargetXDegrees());

                    telemetry.addData("Equation Goal: ", (((-Math.toDegrees(PedroComponent.follower().getHeading()))) * encoderClicksPerDeg));




                    if (lastResult.getFiducialId() == 20 || lastResult.getFiducialId() == 24) {
                        if (lastResult.getTargetXDegrees() < 15 && lastResult.getTargetXDegrees() > -15) {
                            lastHeading = ((-Math.toDegrees(PedroComponent.follower().getHeading())));
                        }
                        lastTurretPose = turretMotor.getCurrentPosition();
                        if (lastResult.getTargetXDegrees() < -5 && (turretControl.getGoal().getPosition() + 12) < 1197){
                            turretControl.setGoal(new KineticState(turretControl.getGoal().getPosition() +12));
                        }
                        else if (lastResult.getTargetXDegrees() > 5 && (turretControl.getGoal().getPosition() - 12) > -1197){
                            turretControl.setGoal(new KineticState(turretControl.getGoal().getPosition() -12));
                        }
                        /*if ((Math.toDegrees(PedroComponent.follower().getHeading()) <= 90 && Math.toDegrees(PedroComponent.follower().getHeading()) >= -90) && (lastResult.getTargetXDegrees() < -1 || lastResult.getTargetXDegrees() > 1)) {
                            telemetry.addData("if", "");
                            turretControl.setGoal(new KineticState(((-Math.toDegrees(PedroComponent.follower().getHeading())) - lastResult.getCameraPoseTargetSpace().getOrientation().getYaw(AngleUnit.DEGREES)) * encoderClicksPerDeg));
                        }
                    }
                    }
                }else {
                if (turretControl.getGoal().getPosition() < 1197 && turretControl.getGoal().getPosition() > -1197) {
                    telemetry.addData("if", "");
                    turretControl.setGoal(new KineticState(( (lastHeading - Math.toDegrees(PedroComponent.follower().getHeading())) * encoderClicksPerDeg)+lastTurretPose));
                }else if ((( (lastHeading - Math.toDegrees(PedroComponent.follower().getHeading())) * encoderClicksPerDeg)+lastTurretPose) > 1196){
                    turretControl.setGoal(new KineticState(1197));
                }else if ((((lastHeading - Math.toDegrees(PedroComponent.follower().getHeading())) * encoderClicksPerDeg)+lastTurretPose) < -1196){
                    turretControl.setGoal(new KineticState(-1197));
                }

                if ((((lastHeading - Math.toDegrees(PedroComponent.follower().getHeading())) * encoderClicksPerDeg)+lastTurretPose) < 1197 && (( (lastHeading - Math.toDegrees(PedroComponent.follower().getHeading())) * encoderClicksPerDeg)+lastTurretPose) > -1197){
                    turretControl.setGoal(new KineticState(( (lastHeading - Math.toDegrees(PedroComponent.follower().getHeading())) * encoderClicksPerDeg)+lastTurretPose));
                }


                telemetry.addData("Flywheel goal: ", flyWheelGoal);
                telemetry.addData("Turret goal: ", turretControl.getGoal().getPosition());
                telemetry.addData("Turret position: ", turretMotor.getCurrentPosition());
                telemetry.addData("locked on: ", lockedOn);
                telemetry.addData("Robot Heading: ", Math.toDegrees(PedroComponent.follower().getHeading()));

                //telemetry.addData("robot Yaw: ", lastResult.getTargetPoseRobotSpace().getOrientation().getYaw(AngleUnit.DEGREES));


            }
        }*/
    }

        public int getIndex (Limelight3A limelight){
            LLResult result = limelight.getLatestResult();
            Map<String, Double> Data = null;

            if (result != null) {
                List<LLResultTypes.FiducialResult> feducialResults = result.getFiducialResults();
                //telemetry.addData("Tx:", feducialResults.get(0).getTargetXDegrees());
                if (result.isValid()) {
                    for (LLResultTypes.FiducialResult tag : feducialResults) {
                        return tag.getFiducialId();
                    }
                }
            }
            return 0;
        }

        @Override
        public void initialize () {
            turretMotor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        @Override
        public void periodic () {
            turretMotor.setPower(turretControl.calculate(turretMotor.getState()));
        }
    }