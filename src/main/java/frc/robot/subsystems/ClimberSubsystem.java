package frc.robot.subsystems;

import frc.robot.Constants.ClimberConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ClimberSubsystem implements Subsystem{

    SparkMax mainMotor;
    SparkMax followerMotor;

    public ClimberSubsystem (){
        mainMotor = new SparkMax(ClimberConstants.mainmotorID, MotorType.kBrushless);
        followerMotor = new SparkMax(ClimberConstants.followerMotorID, MotorType.kBrushless);

        SparkMaxConfig mainConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        mainConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ClimberConstants.currentLimit);
        followerConfig
            .idleMode(IdleMode.kBrake)
            .follow(mainMotor, true);

        mainMotor.configure(mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void climb() {
        mainMotor.set(0.25);
    }

    public void hold() {
        mainMotor.set(0.07);
    }

    public void retract() {
        mainMotor.set(-0.75);
    }

    public void stop() {
        mainMotor.set(0);
    }

}
