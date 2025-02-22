// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DumbSubsystem;
import frc.robot.utils.JoystickUtils;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() 
            // .withDeadband(MaxSpeed * 0.08).withRotationalDeadband(MaxAngularRate * 0.08) //Add a 8% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandJoystick joystickLeft = new CommandJoystick(0);
    private final CommandJoystick joystickRight = new CommandJoystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final DumbSubsystem motor = new DumbSubsystem();
    
    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("Test", autoRoutines::simplePathAuto);
        autoChooser.addRoutine("Score", autoRoutines::scoringAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(JoystickUtils.applyScaler(JoystickUtils.applyDeadzone(-joystickLeft.getY())) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(JoystickUtils.applyScaler(JoystickUtils.applyDeadzone(-joystickLeft.getX())) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(JoystickUtils.applyScaler(JoystickUtils.applyDeadzone(-joystickRight.getX())) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        motor.setDefaultCommand(Commands.run(() -> motor.runMotor(0), motor));

        joystickRight.button(8).whileTrue (Commands.run(() -> motor.runMotor(joystickRight.getZ()), motor));

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystickRight.trigger().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(Rotation2d.kZero)
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystickLeft.button(7).and(joystickRight.button(6)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystickLeft.button(7).and(joystickRight.button(7)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystickLeft.button(7).and(joystickRight.button(10)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystickLeft.button(7).and(joystickRight.button(11)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystickLeft.button(5).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}
