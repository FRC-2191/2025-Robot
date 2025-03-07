package frc.robot;

public final class Constants {

    public static class RobotConstants {
        public static final double kDt = 0.02;
    }

    public static class ElevatorConstants {
        //Elavator climby thingy
        public static final int mainMotorID = 1;
        public static final int followerMotorID = 2;
        public static final int currentLimit = 60;
        

        public static final double kP = 0.125;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0.145;
        public static final double kS = 0.245;
        public static final double kV = 0.3077;
        public static final double kA = 0.005;
        public static final double maxVelocity = 30;
        public static final double maxAcceleration = 60;
        public static final double rotationstoinches = 0.345;
    }

    public static class ClimberConstants {
        //THE CLIMBER OF DOOM
        public static final int mainmotorID = 3;
        public static final int followerMotorID = 4;
    }

    public static class CoralArmConstants {
        //UNDER THE SEA
        public static final int pivotMotorID = 5;
        public static final int intakeMotorID = 6;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double maxVelocity = 0;
        public static final double maxAcceleration = 0;
        public static final double rotationstoinches = 1;
    }

    public static class AlgaeArmConstants {
        //Eat the algae
        public static final int pivotMotorID = 7;
        public static final int intakeMotorID = 8;
        public static final int followerMotorID = 9;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double maxVelocity = 0;
        public static final double maxAcceleration = 0;
        public static final double rotationstoinches = 1;
    }


}