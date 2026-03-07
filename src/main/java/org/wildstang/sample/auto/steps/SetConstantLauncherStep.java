package org.wildstang.sample.auto.steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.launcher.Launcher;

public class SetConstantLauncherStep extends AutoStep{

    public Launcher launcher;
    public boolean active;

     public SetConstantLauncherStep(boolean active){
        this.active = active;
    }
    @Override
    public void initialize() {
        launcher = (Launcher) Core.getSubsystemManager().getSubsystem(WsSubsystems.LAUNCHER);
    }

    @Override
    public void update() {
        launcher.setLauncherState(active);
        setFinished();
    }

    @Override
    public String toString() {
        return "Set Constant Launcher";
    }
}