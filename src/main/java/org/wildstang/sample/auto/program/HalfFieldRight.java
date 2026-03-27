package org.wildstang.sample.auto.program;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.auto.steps.control.AutoStepDelay;
import org.wildstang.framework.auto.steps.control.AutoStepStopAutonomous;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.auto.steps.SetGyroStep;
import org.wildstang.sample.auto.steps.SetIntakeStep;
import org.wildstang.sample.auto.steps.SetLauncherStep;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;


public class HalfFieldRight extends AutoProgram {
    
    /**
     * Defines an AutoProgram's steps, executed in order added.
     * Sleeper uses only a single step: AutoStepStopAutonomous, which
     * does nothing and never finishes.
     */
     @Override
    public void defineSteps() {


        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);

        addStep(new SwervePathFollowerStep("OverRightBump5", swerve, true));

        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        group2.addStep(new SwervePathFollowerStep("RightHalfSwipe", swerve));
        group2.addStep(new SetIntakeStep(true));
        addStep(group2);

        addStep(new AutoStepDelay(500));

        AutoParallelStepGroup group5 = new AutoParallelStepGroup();
        group5.addStep(new SetIntakeStep(false));
        group5.addStep(new SwervePathFollowerStep("AcrossRightBump5", swerve));
        addStep(group5); 

        addStep(new SwervePathFollowerStep("ToShootingSpot5", swerve));
        addStep(new SetLauncherStep());

        addStep(new SwervePathFollowerStep("BackToBegRight", swerve));

        addStep(new SwervePathFollowerStep("OverRightBump5", swerve));

        AutoParallelStepGroup group9 = new AutoParallelStepGroup();
        group9.addStep(new SwervePathFollowerStep("RightHalfSwipe", swerve));
        group9.addStep(new SetIntakeStep(true));
        addStep(group9); 

        addStep(new AutoStepStopAutonomous());

        

    }
    /**
     * Returns the AutoProgram's name.
     * @return The name of the AutoProgram.
     */
    @Override
    public String toString() {
        return "HalfFieldRight";
    }
}
