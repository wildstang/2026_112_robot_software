package org.wildstang.sample.auto.program;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.auto.steps.control.AutoStepStopAutonomous;
import org.wildstang.sample.auto.steps.SetIntakeStep;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.subsystems.intake.Intake;


/**
 * A default AutoProgram that does nothing an never completes.
 * This is used in AutoManager as a final program (program 0) which
 * does nothing but keeps a program running when all programs complete.
 * @author coder65535
 */
public class Program0 extends AutoProgram {

    public Intake intake;
    /**
     * Defines an AutoProgram's steps, executed in order added.
     * Sleeper uses only a single step: AutoStepStopAutonomous, which
     * does nothing and never finishes.
     */
    @Override
    public void defineSteps() {


        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);

        addStep(new AutoStepStopAutonomous());

        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new SwervePathFollowerStep("CrossLeftBump", swerve));

        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        group2.addStep(new SetIntakeStep(true));
        group2.addStep(new SwervePathFollowerStep("toBalls", swerve));

        AutoParallelStepGroup group3 = new AutoParallelStepGroup();
        group3.addStep(new SwervePathFollowerStep("ToRightBump", swerve));

        AutoParallelStepGroup group4 = new AutoParallelStepGroup(); 
        group4.addStep(new SetIntakeStep(false));
        group4.addStep(new SwervePathFollowerStep("CrossRightBump", swerve));

        AutoParallelStepGroup group5 = new AutoParallelStepGroup();
        group5.addStep(new SwervePathFollowerStep("ToShooting", swerve));

        
    }

    /**
     * Returns the AutoProgram's name.
     * @return The name of the AutoProgram.
     */
    @Override
    public String toString() {
        return "Sleeper";
    }
}
