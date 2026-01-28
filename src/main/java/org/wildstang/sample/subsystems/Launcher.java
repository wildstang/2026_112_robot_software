package org.wildstang.sample.subsystems;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsJoystickAxis;
import org.wildstang.hardware.roborio.inputs.WsJoystickButton;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.sample.robot.WsInputs;
import org.wildstang.sample.robot.WsOutputs;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.localization.Localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Launcher implements Subsystem {

    // variables
    public WsSpark launcherMiddle;
    public double launcherSpeed = 0;
    public WsSpark preAccel;
    public double preAccelSpeed = 0;
    public double preAccelSetSpeed = -1;
    public WsSpark hood;
    public double hoodRatio = 1;
    public double hoodP = 1;
    public double hoodSpeed = 0;
    public double hoodSetSpeed = 1;
    public double leftRotation;
    public double rightRotation;
    public WsSpark feed;
    public double feedSetSpeed = -1;
    public double feedSpeed = 0;

    //we will find  these values later when the shooter can shoot, but we need them for our function
    public double a; 
    public double b;

    private Localization localization;
    private Pose2d targetPose;
    private double distance;
    private double speed;

    public WsJoystickAxis r2trigger;
    public WsJoystickButton dPadLeft;
    public WsJoystickButton dPadRight;

    @Override
    public void init() {
        launcherMiddle = (WsSpark) WsOutputs.LAUNCHER_MIDDLE.get();
        preAccel = (WsSpark) WsOutputs.PREACCEL.get();
        feed = (WsSpark) WsOutputs.FEEDER.get();
        hood = (WsSpark) WsOutputs.HOOD.get();

        r2trigger = (WsJoystickAxis) WsInputs.OPERATOR_RIGHT_TRIGGER.get();
        r2trigger.addInputListener(this);
        dPadLeft = (WsJoystickButton) WsInputs.OPERATOR_FACE_LEFT.get();
        dPadLeft.addInputListener(this);
        dPadRight = (WsJoystickButton) WsInputs.OPERATOR_FACE_RIGHT.get();
        dPadRight.addInputListener(this);

        //targetPose = localization.getNearestProcessorPose();
        //distance = localization.getDistanceToHub(targetPose);
        //speed = calculateShooterSpeed(distance);

        //initLoopHood();
    }

    @Override
    public void initSubsystems() {
        localization = (Localization) Core.getSubsystemManager().getSubsystem(WsSubsystems.LOCALIZATION);
    }

    private double getRotationHood() {
        return (hood.getPosition() / hoodRatio) * 2 * Math.PI; // revolutions * 2pi = rad
    }

    private void initLoopHood() {
        hood.initClosedLoop(hoodP, 0, 0, 0);
        hood.configure();
        hood.resetEncoder();
    }

    private void setRotationHood(double rotation) {
        hood.setPosition(((rotation * hoodRatio) / 2) / Math.PI);
    }

    @Override
    public void update() {
        launcherMiddle.setSpeed(launcherSpeed);
        preAccel.setSpeed(preAccelSpeed);
        hood.setSpeed(hoodSpeed);
        feed.setSpeed(feedSpeed);

        SmartDashboard.putNumber("Trigger:", r2trigger.getValue());
        SmartDashboard.putBoolean("Right Button:", dPadRight.getValue());

        SmartDashboard.putNumber("Feed Set Speed:", feedSpeed);
        SmartDashboard.putNumber("Pre-Accel Set Speed:", preAccelSpeed);
        SmartDashboard.putNumber("Launcher Set Speed:", launcherSpeed);
        SmartDashboard.putNumber("Hood Set Speed:", hoodSpeed);

        SmartDashboard.putNumber("Feed Speed:", feed.getVelocity());
        SmartDashboard.putNumber("Pre-Accel Speed:", preAccel.getVelocity());
        SmartDashboard.putNumber("Launcher Speed:", launcherMiddle.getVelocity());
        SmartDashboard.putNumber("Hood Position:", getRotationHood());
    }

    @Override
    public void inputUpdate(Input source) {
        if (source == r2trigger) {
            if (Math.abs(r2trigger.getValue()) >= 0.1) {
                launcherSpeed = Math.abs(r2trigger.getValue());
                // when we get out a and b values we will use this method
                preAccelSpeed = preAccelSetSpeed;
                feedSpeed = feedSetSpeed;
            } else {
                launcherSpeed = 0;
                preAccelSpeed = 0;
                feedSpeed = 0;
            }
        }
        if (source == dPadLeft) {
            hoodSpeed = dPadLeft.getValue() ? hoodSetSpeed : 0;
            // im not sure if we want it so when you hold the dpad longer it moves up more
            // or just when you press the dPad it moves to a certain position. I did it this
            // way but I can switch it pretty easily
        } else if (source == dPadRight) {
            hoodSpeed = dPadRight.getValue() ? -hoodSetSpeed : 0;
        }
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void resetState() {
        feedSpeed = 0;
        preAccelSpeed = 0;
        launcherSpeed = 0;
        hoodSpeed = 0;
    }

    @Override
    public String getName() {
        return "Launcher";
    }
}