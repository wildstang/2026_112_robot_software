package org.wildstang.sample.auto.program;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.auto.steps.control.AutoStepDelay;
import org.wildstang.framework.auto.steps.control.AutoStepStopAutonomous;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.auto.steps.SetLauncherStep;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;


public class ShootMid extends AutoProgram {
    /**
     * Defines an AutoProgram's steps, executed in order added.
     * Sleeper uses only a single step: AutoStepStopAutonomous, which
     * does nothing and never finishes.
     */
    @Override
    public void defineSteps() {


        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);

        addStep(new SwervePathFollowerStep("shootMid", swerve));
        
        addStep(new AutoStepDelay(1000));

        addStep(new SetLauncherStep());
        
        addStep(new AutoStepStopAutonomous());

        
    }

    /**
     * Returns the AutoProgram's name.
     * @return The name of the AutoProgram.
     */
    @Override
    public String toString() {
        return "ShootMid";
    }
}
