package org.wildstang.framework.auto.steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;

public class ResetGyroStep extends AutoStep {

    public ResetGyroStep() {}

    @Override
    public void initialize() {    }

    @Override
    public void update() {
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        swerve.resetGyro();
        setFinished(true);
    }

    @Override
    public String toString() {
        return "Reset Gyro";
    }

}
