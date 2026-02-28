package org.wildstang.sample.auto.steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.launcher.Launcher;

public class SetLauncherStep extends AutoStep{

    public Launcher launcher;

    @Override
    public void initialize() {
        launcher = (Launcher) Core.getSubsystemManager().getSubsystem(WsSubsystems.LAUNCHER);
    }
    @Override
    public void update() {
        launcher.launch();
        setFinished();
    }

    @Override
    public String toString() {
        return "Set Launcher";
    }
}