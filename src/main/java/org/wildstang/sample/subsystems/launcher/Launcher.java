package org.wildstang.sample.subsystems.launcher;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsDPadButton;
import org.wildstang.hardware.roborio.inputs.WsJoystickAxis;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.sample.robot.WsInputs;
import org.wildstang.sample.robot.WsOutputs;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.localization.Localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Launcher implements Subsystem {

    // variables
    private WsSpark launcherMiddle;
    private double targetLauncherVelocity = 500;
    private double launcherSpeed = 0;

    private WsSpark preAccel;
    private double preAccelSetSpeed = -1;
    private double preAccelSpeed = 0;

    private WsSpark hood;
    private double targetHoodAngle = 0;
    private boolean moveHood = false;

    private WsSpark feed;
    private double feedSetSpeed = -1;
    private double feedSpeed = 0;

    //we will find  these values later when the shooter can shoot, but we need them for our function
    private double a; 
    private double b;

    private Localization localization;
    private Pose2d targetPose;
    private double distance;
    private double speed;

    private WsJoystickAxis l2trigger;
    private WsJoystickAxis r2trigger;
    private WsDPadButton dPadLeft;
    private WsDPadButton dPadRight;

    private GenericEntry targetRpm;
    private GenericEntry targetRad;

    @Override
    public void init() {
        launcherMiddle = (WsSpark) WsOutputs.LAUNCHER_MIDDLE.get();

        preAccel = (WsSpark) WsOutputs.PREACCEL.get();
        feed = (WsSpark) WsOutputs.FEEDER.get();
        hood = (WsSpark) WsOutputs.HOOD.get();
        hood.resetEncoder();

        l2trigger = (WsJoystickAxis) WsInputs.OPERATOR_LEFT_TRIGGER.get();
        l2trigger.addInputListener(this);
        r2trigger = (WsJoystickAxis) WsInputs.OPERATOR_RIGHT_TRIGGER.get();
        r2trigger.addInputListener(this);
        dPadLeft = (WsDPadButton) WsInputs.OPERATOR_DPAD_LEFT.get();
        dPadLeft.addInputListener(this);
        dPadRight = (WsDPadButton) WsInputs.OPERATOR_DPAD_RIGHT.get();
        dPadRight.addInputListener(this);

        ShuffleboardTab tab = Shuffleboard.getTab("Launcher");
        targetRpm = tab.add("Launcher Set RPM", targetLauncherVelocity).getEntry();
        targetRad = tab.add("Hood Set Rad", targetHoodAngle).getEntry();
        //targetPose = localization.getNearestProcessorPose();
        //distance = localization.getDistanceToHub(targetPose);
        //speed = calculateShooterSpeed(distance);
    }

    @Override
    public void initSubsystems() {
        localization = (Localization) Core.getSubsystemManager().getSubsystem(WsSubsystems.LOCALIZATION);
    }

    private double getHoodRotation() {
        return LauncherConstants.HOOD_SCALE * (-hood.getPosition() / LauncherConstants.TOTAL_HOOD_RATIO);
    }

    private void setHoodRotation(double rotation) {
        hood.setPosition(LauncherConstants.TOTAL_HOOD_RATIO * (-rotation / LauncherConstants.HOOD_SCALE));
    }

    @Override
    public void update() {
        launcherMiddle.setVelocity(launcherSpeed);
        preAccel.setSpeed(preAccelSpeed);
        feed.setSpeed(feedSpeed);
        if (moveHood) {
            setHoodRotation(targetHoodAngle);
        }

        SmartDashboard.putNumber("Feed Set Speed:", feedSpeed);
        SmartDashboard.putNumber("Pre-Accel Set Speed:", preAccelSpeed);
        SmartDashboard.putBoolean("Move Hood", moveHood);
        targetLauncherVelocity = targetRpm.getDouble(targetLauncherVelocity);
        targetHoodAngle = targetRad.getDouble(targetHoodAngle);
        SmartDashboard.putNumber("Raw Set Hood", LauncherConstants.HOOD_SCALE * (-targetHoodAngle / LauncherConstants.HOOD_SCALE));

        SmartDashboard.putNumber("Feed Speed:", feed.getVelocity());
        SmartDashboard.putNumber("Pre-Accel Speed:", preAccel.getVelocity());
        SmartDashboard.putNumber("Launcher Read RPM:", launcherMiddle.getVelocity());
        SmartDashboard.putNumber("Hood Position:", getHoodRotation());
    }

    @Override
    public void inputUpdate(Input source) {
        if (source == l2trigger) {
            if (Math.abs(l2trigger.getValue()) >= 0.1) {
                launcherSpeed = targetLauncherVelocity;
                preAccelSpeed = preAccelSetSpeed;
            } else {
                launcherSpeed = 0;
                preAccelSpeed = 0;
            }
        }
        if (source == r2trigger) {
            if (Math.abs(r2trigger.getValue()) >= 0.1) {
                feedSpeed = feedSetSpeed;
            } else {
                feedSpeed = 0.5;
            }
        }
        if (source == dPadRight) {
            if (dPadRight.getValue()) {
                moveHood = true;
            } else {
                moveHood = false;
            }
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
        moveHood = false;
    }

    @Override
    public String getName() {
        return "Launcher";
    }
}