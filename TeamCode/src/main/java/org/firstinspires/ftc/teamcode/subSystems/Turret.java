package org.firstinspires.ftc.teamcode.subSystems;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Teleop;

import java.util.List;
import java.util.Map;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    public boolean lockedOn = false;
    private Turret() { }

    private MotorEx yLinear = new MotorEx("ylinear");
    public double flyWheelGoal;
    public static boolean isStarted = false;
    LLResultTypes.FiducialResult lastResult = null;

    KineticState targetStateY = new KineticState();
    KineticState targetStateX = new KineticState();


    private ControlSystem yLinearControl = ControlSystem.builder()
            .posPid(0.007, 0.0, 0.0001)
            .elevatorFF(0)
            .build();

    private ControlSystem xLinearControl = ControlSystem.builder()
            .posPid(0.004, 0.0, 0.0001)
            .elevatorFF(0)
            .build();

    public void lockOnUpdate(Limelight3A limelight, Telemetry telemetry){

        LLResult result = limelight.getLatestResult();


        if (result != null) {

            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> feducialResults =  result.getFiducialResults();
                //telemetry.addData("Tx:", feducialResults.get(0).getTargetXDegrees());
                lastResult = feducialResults.get(0);

                if (lastResult != null){

                    telemetry.addData("Camera Pose Target Space: ", lastResult.getCameraPoseTargetSpace());

                    if(lastResult.getCameraPoseTargetSpace().getPosition().z > -3){
                        flyWheelGoal = 184.4566 * Math.pow(lastResult.getCameraPoseTargetSpace().getPosition().z, 2) + 534.06469 * (lastResult.getCameraPoseTargetSpace().getPosition().z) + 1545.52227;
                    }else if(lastResult.getCameraPoseTargetSpace().getPosition().z < -3 && lastResult.getCameraPoseTargetSpace().getPosition().z > -6){
                        flyWheelGoal = -117.70721 * lastResult.getCameraPoseTargetSpace().getPosition().z + 1157.50788;
                    }

                    telemetry.addData("Function y: ", flyWheelGoal);

                    telemetry.addData("locked on: ", lockedOn);

                    telemetry.addData("robot Yaw: ", lastResult.getTargetPoseRobotSpace().getOrientation().getYaw());

                    if((lastResult.getTargetPoseRobotSpace().getOrientation().getYaw(AngleUnit.DEGREES) < -2 || lastResult.getTargetPoseRobotSpace().getOrientation().getYaw(AngleUnit.DEGREES) < -2) && lockedOn){
                        new TurnBy(Angle.fromDeg(lastResult.getTargetPoseRobotSpace().getOrientation().getYaw(AngleUnit.DEGREES))).schedule();
                    }
                }
            }
        }

        //telemetry.update();

    }

    public int getIndex(Limelight3A limelight){
        LLResult result = limelight.getLatestResult();
        Map<String, Double> Data = null;

        if (result != null) {
            List<LLResultTypes.FiducialResult> feducialResults =  result.getFiducialResults();
            //telemetry.addData("Tx:", feducialResults.get(0).getTargetXDegrees());
            if (result.isValid()) {
                for(LLResultTypes.FiducialResult tag : feducialResults){
                    return tag.getFiducialId();
                }
            }
        }
        return 0;
    }
    public void setYLinear(double ty){
        double encoderClicksPerRev = 1440d / 360d;
        double target =  (encoderClicksPerRev * ty);
        targetStateY = new KineticState(yLinear.getState().getPosition() + target);
        yLinearControl.setGoal(targetStateY);
    }

    @Override
    public void initialize() {

    }
    @Override
    public void periodic() {
    }
}