package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;
import java.util.Map;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

public class TurretLockOn implements Subsystem {
    public static final TurretLockOn INSTANCE = new TurretLockOn();
    public boolean lockedOn = false;
    public boolean zeroToggle = true;
    private static DigitalChannel turretLimitSwitch = null;
    public double encoderClicksPerDeg = 5081 / 360.0; //limits: -1197, 1197

    private TurretLockOn() {
    }

    public MotorEx turretMotor = new MotorEx("turretMotor");

    public double flyWheelGoal;
    public static boolean isStarted = false;
    boolean isBounded;
    boolean willBound;
    public double lastHeading = 0;
    public double lastTurretPose = 0;
    public boolean lockToggle;
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

                    if ((lastResult.getTargetPoseRobotSpace().getOrientation().getYaw(AngleUnit.DEGREES) < -2 || lastResult.getTargetPoseRobotSpace().getOrientation().getYaw(AngleUnit.DEGREES) < -2) && lockedOn ) {
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
        isBounded = (turretControl.getGoal().getPosition() < 1196 && turretControl.getGoal().getPosition() > -1196);
        willBound = ((((lastTurretPose) - (lastHeading - Math.toDegrees(PedroComponent.follower().getHeading()))) * encoderClicksPerDeg) < 1196) && ((((lastTurretPose) - (lastHeading - Math.toDegrees(PedroComponent.follower().getHeading()))) * encoderClicksPerDeg) > -1196);
        telemetry.addData("turretpose: ", turretMotor.getCurrentPosition());
        telemetry.addData("heading: ", Math.toDegrees(PedroComponent.follower().getHeading()));
        telemetry.addData("lastHeading: ", lastHeading);
        telemetry.addData("lastTurretPose: ", lastTurretPose);
        telemetry.addData("lockToggle: ", lockToggle);

        telemetry.addData("heading diff: ", (lastHeading - Math.toDegrees(PedroComponent.follower().getHeading())) * encoderClicksPerDeg);
        if (isBounded) {
            if (result != null) {
                if (result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    lastResult = fiducialResults.get(0);
                    telemetry.addData("tx ", lastResult.getTargetXDegrees());
                    if (lastResult.getFiducialId() == 24) {
                        telemetry.addData("lastResult: ", lastResult);
                        if ((lastResult.getTargetXDegrees() < -5 && (turretMotor.getMotor().getCurrentPosition() - lastResult.getTargetXDegrees() * encoderClicksPerDeg) < 1196)) {
                            turretControl.setGoal(new KineticState(turretMotor.getMotor().getCurrentPosition() - (lastResult.getTargetXDegrees() * encoderClicksPerDeg) * .8));
                            lastHeading = Math.toDegrees(PedroComponent.follower().getHeading());
                        }
                        if (lastResult.getTargetXDegrees() > 5 && (turretMotor.getMotor().getCurrentPosition() - lastResult.getTargetXDegrees() * encoderClicksPerDeg) > -1196) {
                            turretControl.setGoal(new KineticState(turretMotor.getMotor().getCurrentPosition() - (lastResult.getTargetXDegrees() * encoderClicksPerDeg) * 0.8));
                            lastHeading = Math.toDegrees(PedroComponent.follower().getHeading());
                        }
                        if (lastResult.getTargetXDegrees() < 13 && lastResult.getTargetXDegrees() > -13) {
                            lastHeading = Math.toDegrees(PedroComponent.follower().getHeading());
                        }
//blah

                    }
                    if (lastResult.getFiducialId() != 24){
                        if ((((lastHeading - Math.toDegrees(PedroComponent.follower().getHeading())) * encoderClicksPerDeg) + turretControl.getGoal().getPosition()) < 1196 && (((lastHeading - Math.toDegrees(PedroComponent.follower().getHeading())) * encoderClicksPerDeg) + turretControl.getGoal().getPosition()) > -1196) {
                            turretControl.setGoal(new KineticState(((lastHeading - Math.toDegrees(PedroComponent.follower().getHeading())) * encoderClicksPerDeg) + turretMotor.getMotor().getCurrentPosition()));
                        }
                    }
                }
                if (!result.isValid()){
                    if ((((lastHeading - Math.toDegrees(PedroComponent.follower().getHeading())) * encoderClicksPerDeg) + turretControl.getGoal().getPosition()) < 1196 && (((lastHeading - Math.toDegrees(PedroComponent.follower().getHeading())) * encoderClicksPerDeg) + turretControl.getGoal().getPosition()) > -1196) {
                        turretControl.setGoal(new KineticState(((lastHeading - Math.toDegrees(PedroComponent.follower().getHeading())) * encoderClicksPerDeg) + turretMotor.getMotor().getCurrentPosition()));
                    }
                }

        if (!isBounded) {
            turretControl.setGoal(new KineticState(0));
        }




            }
        }
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