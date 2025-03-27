package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private final RobotContainer m_container;

    public AutoRoutines(AutoFactory factory, RobotContainer container) {
        m_factory = factory;
        m_container = container;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine testRoutine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = testRoutine.trajectory("Center");
    
        testRoutine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        

        simplePath.done().onTrue(m_container.generateSuperstructureCommand(
            new State (ElevatorConstants.l4, 0),
            new State (-50, 0),
            new State(-30, 0))
            .withDeadline(Commands.waitSeconds(5))
            .andThen(m_container.generateCoralOuttakeCommand().withTimeout(.75)));
        return testRoutine;
    }

    public AutoRoutine rightAuto() {
        
        final AutoRoutine scoringRoutine = m_factory.newRoutine("ScorePath auto");
        final AutoTrajectory scoring = scoringRoutine.trajectory("Right");

        scoringRoutine.active().onTrue(
            scoring.resetOdometry()
                .andThen(scoring.cmd())
        );

        scoring.done().onTrue(m_container.generateSuperstructureCommand(
            new State (ElevatorConstants.l4, 0),
            new State (-50, 0),
            new State(-30, 0))
            .withDeadline(Commands.waitSeconds(5))
            .andThen(m_container.generateCoralOuttakeCommand().withTimeout(.75)));
        
        return scoringRoutine;
    }
    public AutoRoutine leftAuto() {
        
        final AutoRoutine leftScoringRoutine = m_factory.newRoutine("LeftScorePath auto");
        final AutoTrajectory scoring = leftScoringRoutine.trajectory("Left");

        leftScoringRoutine.active().onTrue(
            scoring.resetOdometry()
                .andThen(scoring.cmd())
        );

        scoring.done().onTrue(m_container.generateSuperstructureCommand(
            new State (ElevatorConstants.l4, 0),
            new State (-50, 0),
            new State(-30, 0))
            .withDeadline(Commands.waitSeconds(5))
            .andThen(m_container.generateCoralOuttakeCommand().withTimeout(.75)));
        
        return leftScoringRoutine;
    }
}