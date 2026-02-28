package org.wildstang.sample.auto.program;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.auto.steps.control.AutoStepStopAutonomous;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.auto.steps.SetIntakeStep;
import org.wildstang.sample.auto.steps.SetLauncherStep;
import org.wildstang.sample.auto.steps.SetGyroStep;


public class HalfFieldLeft extends AutoProgram {
    
    /**
     * Defines an AutoProgram's steps, executed in order added.
     * Sleeper uses only a single step: AutoStepStopAutonomous, which
     * does nothing and never finishes.
     */
     @Override
    public void defineSteps() {


        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);

        addStep(new SetGyroStep(270.0));
      
        AutoParallelStepGroup group0 = new AutoParallelStepGroup();
        group0.addStep(new SetLauncherStep());
        addStep(group0);


        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new SwervePathFollowerStep("OverLeftBump2", swerve));
        addStep(group1);

        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        group2.addStep(new SwervePathFollowerStep("ToMidBalls2", swerve));
        group2.addStep(new SetIntakeStep(true));
        addStep(group2);

        AutoParallelStepGroup group3 = new AutoParallelStepGroup();
        group3.addStep(new SwervePathFollowerStep("ToLeftBump2", swerve));
        addStep(group3);

        AutoParallelStepGroup group4 = new AutoParallelStepGroup();
        group4.addStep(new SwervePathFollowerStep("AcrossLeftBump2", swerve));
        group4.addStep(new SetIntakeStep(false));
        addStep(group4);

        AutoParallelStepGroup group5 = new AutoParallelStepGroup();
        group5.addStep(new SwervePathFollowerStep("ToShootingSpotLeft2", swerve));
        group5.addStep(new SetLauncherStep());
        addStep(group5);

        AutoParallelStepGroup group6 = new AutoParallelStepGroup();
        group3.addStep(new SwervePathFollowerStep("BackToBeg", swerve));
        addStep(group6);

        //second run
         AutoParallelStepGroup group7 = new AutoParallelStepGroup();
        group7.addStep(new SwervePathFollowerStep("OverLeftBump2", swerve));
        addStep(group7);

        AutoParallelStepGroup group8 = new AutoParallelStepGroup();
        group8.addStep(new SwervePathFollowerStep("ToMidBalls2", swerve));
        group8.addStep(new SetIntakeStep(true));
        addStep(group8);

        AutoParallelStepGroup group9 = new AutoParallelStepGroup();
        group9.addStep(new SwervePathFollowerStep("ToLeftBump2", swerve));
        addStep(group9);

        AutoParallelStepGroup group10 = new AutoParallelStepGroup();
        group10.addStep(new SwervePathFollowerStep("AcrossLeftBump2", swerve));
        group10.addStep(new SetIntakeStep(false));
        addStep(group10);

        AutoParallelStepGroup group11 = new AutoParallelStepGroup();
        group11.addStep(new SwervePathFollowerStep("ToShootingSpotLeft2", swerve));
        group11.addStep(new SetLauncherStep());
        addStep(group11);

        AutoParallelStepGroup group12 = new AutoParallelStepGroup();
        group12.addStep(new SwervePathFollowerStep("ToClimb2", swerve));
// group12.addStep(new SetClimbStep(true));
        addStep(group12);

        addStep(new AutoStepStopAutonomous());

    }
    /**
     * Returns the AutoProgram's name.
     * @return The name of the AutoProgram.
     */
    @Override
    public String toString() {
        return "HalfFieldLeft";
    }
}
