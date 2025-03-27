package frc.robot;

public final class Constants {

    public static class RobotConstants {
        public static final double kDt = 0.02;        
    }

    public static class ElevatorConstants {
        //Elavator thingy
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
        public static final double maxVelocity = 20;
        public static final double maxAcceleration = 30;
        public static final double rotationstoinches = 0.345;

        public static final double resting = 2;
        public static final double l2 = 4.65;
        public static final double l3 = 19.57;
        public static final double l4 = 43.61;
        public static final double net = 64;
        public static final double feederStation = 1;

    }

    public static class CoralArmConstants {
        //UNDER THE SEA
        public static final int pivotMotorID = 5;
        public static final int intakeMotorID = 6;

        public static final int pivotCurrentLimit = 50;
        public static final int intakeCurrentLimit = 75;

        public static final double kP = 0.004;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0.05;
        public static final double kS = 0.15;
        public static final double kV = 0.02358;
        public static final double kA = 0;
        public static final double maxVelocity = 180;
        public static final double maxAcceleration = 720;
        public static final double sensorToMechanismRatio = 360;
        public static final double absoluteOffset = 0.1774955;

        public static final double resting = -45;
        public static final double diagonalBranch = -30;
        public static final double l4 = -45;
        public static final double feederStation = 3;
        public static final double tucked = -87;
        public static final double net = 85;
    }

    public static class AlgaeArmConstants {
        //Eat the algae
        public static final int pivotMotorID = 7;
        public static final int intakeMotorID = 8;
        public static final int followerMotorID = 9;

        public static final int pivotCurrentLimit = 50;
        public static final int intakeCurrentLimit = 50;

        public static final double kP = 0.005;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0.1;
        public static final double kS = 0.25;
        public static final double kV = 0.02358;
        public static final double kA = 0;
        public static final double maxVelocity = 180;
        public static final double maxAcceleration = 360;
        public static final double sensorToMechanismRatio = 360;
        public static final double absoluteOffset = 0.5062869;

        public static final double resting = -50;
        public static final double reef = -30;
        public static final double l4 = -50;
        public static final double feederStation = -60;
        public static final double tucked = -80;
        public static final double net = 20;
        public static final double intercept = -10;
    }
}
