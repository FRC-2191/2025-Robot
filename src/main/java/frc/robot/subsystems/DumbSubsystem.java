package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class DumbSubsystem implements Subsystem{

    SparkMax dumbo;

    public DumbSubsystem (){
        dumbo = new SparkMax(20, MotorType.kBrushless);
        SparkMaxConfig  config = new SparkMaxConfig();
        config.inverted(true);
        dumbo.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void runMotor (double speed) {
        dumbo.set(speed);
    }

}
