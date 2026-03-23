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

    private Turret() {}

    public static synchronized Turret getInstance(Limelight3A l, Telemetry tel)
    {
        if (single_instance == null)
            single_instance = new Turret();

        limelight = l;
        telemetry = tel;
        return single_instance;
    }

    public MotorEx turretMotor = new MotorEx("turretMotor");
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
        Pose2D targetPosition;
        Pose botPosePedro = new Pose(0,0,0); // left as zero to indicate values that should be put in later


        PedroComponent.follower().setPose(botPosePedro); // update localizer

        switch(lastResult.getFiducialId()){
            case 22:
                targetPosition = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0); //set to april tag postition coordinates later.
                break;
            case 21:
                targetPosition = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0); //set to april tag postition coordinates later.
                break;
            default:
                return; // don't run anymore code if the motif is detected
        }
        double encoderTarget = MathUtils.clamp(0, -1197, 1197); // todo: trig to figure out what the heck



        turretControl.setGoal(new KineticState(encoderTarget));


    }
    public void lockOnUpdate() {

        checkForValidTag();
        //telemetry.update();

    }


    @Override
    public void initialize () {
    }

    @Override
    public void periodic () {
        if (opModeIsStarted) {
            turretMotor.setPower(turretControl.calculate(turretMotor.getState()));
        }
    }
}