package frc.robot.subsystems;

import frc.robot.Constants.CoralArmConstants;
import frc.robot.Constants.RobotConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CoralSubsystem implements Subsystem{
    SparkFlex pivotMotor;
    SparkFlex intakeMotor;
    SparkFlex followerMotor;
    AbsoluteEncoder encoder;

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State currentState = new TrapezoidProfile.State();

    TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                    CoralArmConstants.maxVelocity,
                    CoralArmConstants.maxAcceleration));

    ArmFeedforward feedforward = new ArmFeedforward(
        CoralArmConstants.kS,
        CoralArmConstants.kG,
        CoralArmConstants.kV,
        CoralArmConstants.kA);

    public CoralSubsystem() {
        pivotMotor = new SparkFlex(CoralArmConstants.pivotMotorID, MotorType.kBrushless);
        intakeMotor = new SparkFlex(CoralArmConstants.intakeMotorID, MotorType.kBrushless);
        encoder = pivotMotor.getAbsoluteEncoder();
        SparkFlexConfig pivotConfig = new SparkFlexConfig();
        SparkFlexConfig intakeConfig = new SparkFlexConfig();

        pivotConfig
            .smartCurrentLimit(CoralArmConstants.pivotCurrentLimit)
            .idleMode(IdleMode.kBrake)
            .inverted(true);
        pivotConfig.absoluteEncoder.positionConversionFactor(CoralArmConstants.sensorToMechanismRatio)
            .velocityConversionFactor(6)
            .zeroOffset(CoralArmConstants.absoluteOffset)
            .inverted(true)
            .zeroCentered(true);
        pivotConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .positionWrappingInputRange(-180, 180)
                    .positionWrappingEnabled(true)
                    .pid(CoralArmConstants.kP, CoralArmConstants.kI, CoralArmConstants.kD);

        intakeConfig
            .smartCurrentLimit(CoralArmConstants.intakeCurrentLimit)
            .idleMode(IdleMode.kBrake)
            .inverted(true);


        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        currentState.position = encoder.getPosition();
    }


    public void setVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
    }

    public void setGoal(TrapezoidProfile.State newGoal) {
        goal = newGoal;
        setpoint = profile.calculate(RobotConstants.kDt, currentState, goal);
    }

    public void moveToGoal() {
        setpoint = profile.calculate(RobotConstants.kDt, setpoint, goal);
        pivotMotor.getClosedLoopController().setReference(
            setpoint.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforward.calculate(Rotation2d.fromDegrees(encoder.getPosition()).getRadians(), setpoint.velocity),
            ArbFFUnits.kVoltage);
    }

    public void intake() {
        intakeMotor.set(0.6);
    }

    public void outtake() {
        intakeMotor.set(-1);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        currentState.position = encoder.getPosition();
        currentState.velocity = encoder.getVelocity();
        SmartDashboard.putNumber("Coral Pivot Position", currentState.position);
        SmartDashboard.putNumber("Coral Pivot Setpoint", setpoint.position);
        SmartDashboard.putNumber("Coral Pivot Voltage", pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput());
    }
}
