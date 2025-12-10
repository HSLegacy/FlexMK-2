package org.firstinspires.ftc.teamcode.subSystems;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.teamcode.Teleop;

import java.util.List;
import java.util.Map;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    boolean lockedOn = false;
    private Turret() { }

    private MotorEx yLinear = new MotorEx("ylinear");
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
    public void lockOn(){
        lockedOn = true;
    }
    public void lockOff(){
        lockedOn = false;
    }
    public void lockOnUpdate(Limelight3A limelight){

            LLResult result = limelight.getLatestResult();


            if (result != null && lockedOn == true) {


                if (result.isValid()) {
                    List<LLResultTypes.FiducialResult> feducialResults =  result.getFiducialResults();
                    //telemetry.addData("Tx:", feducialResults.get(0).getTargetXDegrees());
                    lastResult = feducialResults.get(0);

                    if (lastResult != null){
                        if(lastResult.getTargetXDegrees() < -2 && lockedOn){
                            follower().turnDegrees(Math.abs(lastResult.getTargetXDegrees()), true);
                        }
                        else if (lastResult.getTargetXDegrees() > 2 && lockedOn){
                            follower().turnDegrees(Math.abs(lastResult.getTargetXDegrees()), false);
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