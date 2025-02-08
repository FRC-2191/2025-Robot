package frc.robot.utils;

public abstract class JoystickUtils {
    private static final double deadzone = 0.1;

    public static double applyDeadzone (double input) {
        if (Math.abs(input) < deadzone) {
            return 0;
        }
        return ((Math.abs(input) - deadzone)/(1 - deadzone)) * Math.signum(input);
    }

    public static double applyScaler (double input) {
        return (0.7 * input + 0.3 * Math.pow(input, 3));
    }
}