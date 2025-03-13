package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ElevatorSubsystem implements Subsystem{
    SparkFlex motor1;
    SparkFlex motor2;
    RelativeEncoder encoder;

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    ElevatorFeedforward feedforward = new ElevatorFeedforward(
        ElevatorConstants.kS,
        ElevatorConstants.kG,
        ElevatorConstants.kV,
        ElevatorConstants.kA);

    TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                    ElevatorConstants.maxVelocity,
                    ElevatorConstants.maxAcceleration));
    
    public ElevatorSubsystem(){
        motor1 = new SparkFlex(ElevatorConstants.mainMotorID, MotorType.kBrushless);
        motor2 = new SparkFlex(ElevatorConstants.followerMotorID, MotorType.kBrushless);
        encoder = motor1.getEncoder();
        SparkFlexConfig mainConfig = new SparkFlexConfig();
        SparkFlexConfig followerConfig = new SparkFlexConfig();

        mainConfig
            .smartCurrentLimit(ElevatorConstants.currentLimit)
            .idleMode(IdleMode.kBrake)
            .inverted(true);
        mainConfig.encoder.positionConversionFactor(ElevatorConstants.rotationstoinches);
        mainConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

        followerConfig
            .idleMode(IdleMode.kBrake)
            .follow(motor1, true);


        motor1.configure(mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public void setVoltage(double voltage) {
        motor1.setVoltage(voltage);
    }

    public void setGoal(TrapezoidProfile.State newGoal) {
        goal = newGoal;
    }

    public void moveToGoal() {
        setpoint = profile.calculate(RobotConstants.kDt, setpoint, goal);
        int acceleration_direction = 0;
        if (setpoint.velocity < ElevatorConstants.maxVelocity) {
            if (profile.timeLeftUntil(goal.position) / profile.totalTime() < 0.5) {
                acceleration_direction = -1;
            }
            else {
                acceleration_direction = 1;
            }
        }
        motor1.getClosedLoopController().setReference(
            setpoint.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforward.calculate(setpoint.velocity, ElevatorConstants.maxAcceleration * acceleration_direction),
            ArbFFUnits.kVoltage);
    }

    public Trigger atL2() {
        return new Trigger(() -> goal.position == ElevatorConstants.l2);
    }
    public Trigger atL3() {
        return new Trigger(() -> goal.position == ElevatorConstants.l3);
    }
    public Trigger atL4() {
        return new Trigger(() -> goal.position == ElevatorConstants.l4);
    }
    public Trigger atRest() {
        return new Trigger(() -> goal.position == ElevatorConstants.resting);
    }
    public Trigger atNet() {
        return new Trigger(() -> goal.position == ElevatorConstants.net);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
        SmartDashboard.putNumber("Elevator Setpoint", setpoint.position);
        SmartDashboard.putNumber("Elevator Voltage", motor1.getBusVoltage() * motor1.getAppliedOutput());
    }
}
