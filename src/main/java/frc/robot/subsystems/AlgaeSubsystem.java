package frc.robot.subsystems;

import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.RobotConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AlgaeSubsystem implements Subsystem{
    SparkFlex pivotMotor;
    SparkFlex intakeMotor;
    SparkFlex followerMotor;
    RelativeEncoder encoder;

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    ArmFeedforward feedforward = new ArmFeedforward(
        AlgaeArmConstants.kS,
        AlgaeArmConstants.kG,
        AlgaeArmConstants.kV,
        AlgaeArmConstants.kA);

    TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                    AlgaeArmConstants.maxVelocity,
                    AlgaeArmConstants.maxAcceleration));


    public void setVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
        System.out.println(voltage);
    }

    public void setGoal(TrapezoidProfile.State newGoal) {
        goal = newGoal;
    }

    public void moveToGoal() {
        setpoint = profile.calculate(RobotConstants.kDt, setpoint, goal);
        int acceleration_direction = 0;
        if (setpoint.velocity < AlgaeArmConstants.maxVelocity) {
            if (profile.timeLeftUntil(goal.position) / profile.totalTime() < 0.5) {
                acceleration_direction = -1;
            }
            else {
                acceleration_direction = 1;
            }
        }
        pivotMotor.getClosedLoopController().setReference(
            setpoint.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforward.calculate(setpoint.velocity, AlgaeArmConstants.maxAcceleration * acceleration_direction),
            ArbFFUnits.kVoltage);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
        SmartDashboard.putNumber("Elevator Setpoint", setpoint.position);
        SmartDashboard.putNumber("Elevator Voltage", pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput());
    }
}
