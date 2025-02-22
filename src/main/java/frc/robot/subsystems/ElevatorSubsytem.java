package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ElevatorSubsytem implements Subsystem{
    SparkFlex motor1;
    SparkFlex motor2;
    RelativeEncoder encoder;
    
    public ElevatorSubsytem(int motor1ID, int motor2ID){
        motor1 = new SparkFlex(motor1ID, MotorType.kBrushless);
        motor2 = new SparkFlex(motor2ID, MotorType.kBrushless);
        encoder = motor1.getEncoder();
        SparkFlexConfig mainConfig = new SparkFlexConfig();
        SparkFlexConfig followerConfig = new SparkFlexConfig();

        mainConfig.smartCurrentLimit(ElevatorConstants.currentLimit);
        mainConfig.encoder.positionConversionFactor(ElevatorConstants.rotationstoinches);
        mainConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

        followerConfig.follow(motor1, true);

        motor1.configure(mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
    }
}
