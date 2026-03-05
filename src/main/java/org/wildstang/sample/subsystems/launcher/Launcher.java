package org.wildstang.sample.subsystems.launcher;

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
import org.wildstang.sample.subsystems.localization.LocalizationConstants;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Launcher implements Subsystem {

    // variables
    private WsSpark launcherMiddle;
    private double targetLauncherVelocity = 2500;
    private boolean runLauncher = false;

    private WsSpark preAccel;
    private double preAccelSetSpeed = 1;

    private WsSpark hood;
    private double targetHoodAngle = 0;
    private double hoodAngleOnPress = 0;

    private WsSpark feed;
    private double feedSetSpeed = 1;
    private double feedSpeed = 0;

    private Localization localization;
    private SwerveDrive drive;

    private WsJoystickAxis l2trigger;
    private WsJoystickAxis r2trigger;
    private WsJoystickButton bButton;

    private GenericEntry targetRpm;
    private GenericEntry targetRad;

    @Override
    public void init() {
        launcherMiddle = (WsSpark) WsOutputs.LAUNCHER_MIDDLE.get();

        preAccel = (WsSpark) WsOutputs.PREACCEL.get();
        feed = (WsSpark) WsOutputs.FEEDER.get();
        hood = (WsSpark) WsOutputs.HOOD.get();
        hood.resetEncoder();

        l2trigger = (WsJoystickAxis) WsInputs.DRIVER_LEFT_TRIGGER.get();
        l2trigger.addInputListener(this);
        r2trigger = (WsJoystickAxis) WsInputs.DRIVER_RIGHT_TRIGGER.get();
        r2trigger.addInputListener(this);
        bButton = (WsJoystickButton) WsInputs.DRIVER_FACE_RIGHT.get();
        bButton.addInputListener(this);

        ShuffleboardTab tab = Shuffleboard.getTab("Launcher");
        targetRpm = tab.add("Launcher Set RPM", targetLauncherVelocity).getEntry();
        targetRad = tab.add("Hood Set Degrees", targetHoodAngle).getEntry();
    }

    @Override
    public void initSubsystems() {
        localization = (Localization) Core.getSubsystemManager().getSubsystem(WsSubsystems.LOCALIZATION);
        drive = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
    }

    private double getHoodRotation() {
        return LauncherConstants.HOOD_SCALE * (-hood.getPosition() / LauncherConstants.TOTAL_HOOD_RATIO);
    }

    private void setHoodRotation(double rotation) {
        hood.setPosition(LauncherConstants.TOTAL_HOOD_RATIO * (-rotation / LauncherConstants.HOOD_SCALE));
    }

    private double calculateLauncherSpeed(double distance) {
        //return targetLauncherVelocity;
        return 330.7127 * distance + 710.1222;
    }

    private double calculateHoodAngle(double distance) {
        //return targetHoodAngle;
        double curX = localization.getCurrentPose().getX();
        if (Core.isBlueAlliance() && curX < LocalizationConstants.BLUE_HUB_X) {
            return 0;
        } else if (!Core.isBlueAlliance() && curX > LocalizationConstants.RED_HUB_X) {
            return 0;
        } else {
            return -2.06313 * distance + 47.06313;
        }
    }

    @Override
    public void update() {
        double distance = localization.getAllianceHubDistance();
        double launcherSpeed = calculateLauncherSpeed(distance);
        double hoodAngle = calculateHoodAngle(targetHoodAngle);

        if (runLauncher) {
            launcherMiddle.setVelocity(launcherSpeed);
            preAccel.setSpeed(preAccelSetSpeed);
            setHoodRotation(hoodAngle);
        }
        else {
            launcherMiddle.setVelocity(0);
            preAccel.setSpeed(0);
        }

        if (feedSpeed != 0) {
            feed.setSpeed(feedSpeed);
        }
        else if (launcherMiddle.getVelocity() > launcherSpeed - 100 || drive.getVisionOverride()) {
            feed.setSpeed(feedSetSpeed);
        }
        else {
            feed.setSpeed(0);
        }

        SmartDashboard.putNumber("Hub Distance (m)", distance);
        SmartDashboard.putNumber("Angle Distance (rad)", localization.getAllianceHubAngle());
        SmartDashboard.putNumber("Target Launcher RPM", launcherSpeed);
        SmartDashboard.putNumber("Target Hood Angle (Deg)", hoodAngle);

        SmartDashboard.putNumber("Feed Duty Cycle", feedSpeed);
        SmartDashboard.putBoolean("Run Launcher", runLauncher);

        SmartDashboard.putNumber("Feed Read RPM", feed.getVelocity());
        SmartDashboard.putNumber("Pre-Accel Read RPM", preAccel.getVelocity());
        SmartDashboard.putNumber("Launcher Read RPM", launcherMiddle.getVelocity());
        SmartDashboard.putNumber("Hood Position (Deg)", getHoodRotation());

        targetLauncherVelocity = targetRpm.getDouble(targetLauncherVelocity);
        targetHoodAngle = targetRad.getDouble(targetHoodAngle);
    }

    @Override
    public void inputUpdate(Input source) {
        if (source == r2trigger) {
            boolean hasInput = Math.abs(r2trigger.getValue()) >= 0.1;

            if (hasInput && !runLauncher) {
                runLauncher = true;
                //drive.setDriveState(SwerveDrive.DriveState.LAUNCH);
            } else if (!hasInput && runLauncher) {
                runLauncher = false;
                drive.setDriveState(SwerveDrive.DriveState.TELEOP);
            }
        }
        else if (source == l2trigger) {
            if (Math.abs(l2trigger.getValue()) >= 0.5) {
                feedSpeed = -feedSetSpeed;
            }
            else {
                feedSpeed = 0;
            }
        }
        else if (source == bButton) {
            SparkMaxConfig config = LauncherConstants.hoodConfig();
            if (bButton.getValue()) {
                hoodAngleOnPress = getHoodRotation();
                config.idleMode(IdleMode.kCoast);
            } else if (Math.abs(hoodAngleOnPress - getHoodRotation()) > 3) {
                hood.resetEncoder();
                config.idleMode(IdleMode.kBrake);
            }
            hood.configure(config);
        }

        if (source != r2trigger && !runLauncher) {

        }
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void resetState() {
        feedSpeed = 0;
        runLauncher = false;
    }

    @Override
    public String getName() {
        return "Launcher";
    }
}