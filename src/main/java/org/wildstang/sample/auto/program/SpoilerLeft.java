package org.wildstang.sample.auto.program;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.AutoSerialStepGroup;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.auto.steps.control.AutoStepDelay;
import org.wildstang.framework.auto.steps.control.AutoStepStopAutonomous;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.auto.steps.SetIntakeStep;
import org.wildstang.sample.auto.steps.SetLauncherStep;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;


public class SpoilerLeft extends AutoProgram {
    
    /**
     * Defines an AutoProgram's steps, executed in order added.
     * Sleeper uses only a single step: AutoStepStopAutonomous, which
     * does nothing and never finishes.
     */
     @Override
    public void defineSteps() {


        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);

        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new SwervePathFollowerStep("SpoilerLeft1", swerve));
        AutoSerialStepGroup group1sub1 = new AutoSerialStepGroup();
        group1sub1.addStep(new AutoStepDelay(1800));
        group1sub1.addStep(new SetIntakeStep(true));
        group1.addStep(group1sub1);
        addStep(group1);

        addStep(new SwervePathFollowerStep("SpoilerLeft2", swerve));

        // addStep(new AutoStepDelay(500));

        // AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        // group2.addStep(new SetIntakeStep(false));
        // group2.addStep(new SwervePathFollowerStep("AcrossLeftBump5", swerve));
        // addStep(group2); 

        // addStep(new SwervePathFollowerStep("ToShootingSpot5", swerve));
        // addStep(new AutoStepDelay(800));
        // addStep(new SetLauncherStep());

        // addStep(new SwervePathFollowerStep("BackToBegLeft", swerve));

        // addStep(new SwervePathFollowerStep("OverLeftBump5", swerve));

        // AutoParallelStepGroup group3 = new AutoParallelStepGroup();
        // group3.addStep(new SwervePathFollowerStep("LeftHalfSwipe", swerve));
        // group3.addStep(new SetIntakeStep(true));
        // addStep(group3); 

        addStep(new AutoStepStopAutonomous());

        

    }
    /**
     * Returns the AutoProgram's name.
     * @return The name of the AutoProgram.
     */
    @Override
    public String toString() {
        return "SpoilerLeft";
    }
}
