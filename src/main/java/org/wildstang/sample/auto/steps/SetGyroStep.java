package org.wildstang.sample.auto.steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;


public class SetGyroStep extends AutoStep{

    public double angle; 
    public SwerveDrive swerve;

    public SetGyroStep(double angle){
        this.angle = angle;
    }

    @Override
    public void initialize() {}

    @Override
    public void update() {
      if( Core.isBlueAlliance() ) {
        swerve.setGyro(angle);
      } else {
        swerve.setGyro(angle + Math.PI);
      }
        setFinished();
    }

    @Override
    public String toString() {
        return "Set Gyro";
    }
}