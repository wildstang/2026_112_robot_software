package org.wildstang.sample.auto.steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.launcher.Launcher;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.Timer;


public class SetLauncherStep extends AutoStep{

    private Launcher launcher;
    private SwerveDrive swerve;
    private Timer timer = new Timer();

    @Override
    public void initialize() {
        launcher = (Launcher) Core.getSubsystemManager().getSubsystem(WsSubsystems.LAUNCHER);
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        timer.reset();
        timer.start();       
        launcher.startLaunch();

    }
    @Override
    public void update() {
        if (timer.get() >= 8.0){
            launcher.stopLaunch();
            swerve.resetState();
            setFinished();
        }
    }

    @Override
    public String toString() {
        return "Set Launcher";
    }
}