package org.wildstang.sample.auto.steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.intake.Intake;

public class SetIntakeStep extends AutoStep{

    public Intake intake; 
    public boolean deployed;

    public SetIntakeStep(boolean deployed){
        this.deployed = deployed;
    }

    @Override
    public void initialize() {
        intake = (Intake) Core.getSubsystemManager().getSubsystem(WsSubsystems.INTAKE);
    }

    @Override
    public void update() {
        if(deployed) {
            intake.deployIntake();
        } else {
            intake.removeIntake();
        }
        setFinished();
    }

    @Override
    public String toString() {
        return "Set Intake";
    }
}