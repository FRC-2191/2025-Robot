package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine testRoutine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = testRoutine.trajectory("Test");
    
        testRoutine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return testRoutine;
    }

    public AutoRoutine scoringAuto() {
        
        final AutoRoutine scoringRoutine = m_factory.newRoutine("ScorePath auto");
        final AutoTrajectory scoring = scoringRoutine.trajectory("scoring");

        scoringRoutine.active().onTrue(
            scoring.resetOdometry()
                .andThen(scoring.cmd())
        );
        return scoringRoutine;
    }
}