package org.firstinspires.ftc.teamcode.utilities;

public class MathUtils {
    // Clamp function for integers
    public static int clamp(int value, int min, int max) {
        if (min > max) {
            throw new IllegalArgumentException("Min value must be less than or equal to max value");
        }
        return Math.max(min, Math.min(max, value));
    }
}
