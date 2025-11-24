package org.firstinspires.ftc.teamcode.subSystems;

import com.bylazar.configurables.annotations.Configurable;

import java.util.function.Function;

/**
 * This is the PIDFCoefficients class. This class handles holding coefficients for PIDF
 * controllers.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Havish Sripada - 12808 RevAmped Robotics
 * @version 1.0, 3/5/2024
 */
@Configurable
public class FlyWheelConstants{

    public static double P = 0.1d;
    public static double I = 0.0d;
    public static double D = 0.0d;


}