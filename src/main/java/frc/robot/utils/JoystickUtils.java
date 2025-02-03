package frc.robot.utils;

public abstract class JoystickUtils {
    private static final double deadzone = 0.01;

    public static double applyDeadzone (double input) {
        if (Math.abs(input) < 0.1) {
            return 0;
        }
        return ((Math.abs(input) - deadzone)/(1 - deadzone)) * Math.signum(input);
    }

    public static double applyScaler (double input) {
        return (.25 * input + .75 * Math.pow(input, 3));
    }
}
