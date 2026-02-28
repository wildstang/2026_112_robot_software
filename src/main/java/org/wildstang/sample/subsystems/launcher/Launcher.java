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
import org.wildstang.sample.subsystems.intake.Intake;
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
    private double newFeedSpeed = 0;

    private Localization localization;
    private SwerveDrive drive;
    private Intake intake;

    private WsJoystickAxis l2trigger;
    private WsJoystickAxis r2DriverTrigger;
    private WsJoystickAxis r2OperatorTrigger;
    private WsJoystickButton bButton;
    private WsJoystickButton driverYButton;
    private WsJoystickButton operatorXButton;

    private GenericEntry targetLauncherVelocityEntry;
    private GenericEntry targetHoodAngleEntry;
    private GenericEntry feedingModeAngleEntry;
    private GenericEntry feedingModeRpmEntry;

    private boolean operatorMode = false;
    private boolean resetHood = false;
    private boolean isFeedMode = false;
    private boolean shouldFeed = false;
    private boolean overrideFeedInput = false;

    private double feedingModeAngle = 30;
    private double feedingModeRpm = 3400;

    @Override
    public void init() {
        launcherMiddle = (WsSpark) WsOutputs.LAUNCHER_MIDDLE.get();

        preAccel = (WsSpark) WsOutputs.PREACCEL.get();
        feed = (WsSpark) WsOutputs.FEEDER.get();
        hood = (WsSpark) WsOutputs.HOOD.get();
        hood.resetEncoder();

        l2trigger = (WsJoystickAxis) WsInputs.DRIVER_LEFT_TRIGGER.get();
        l2trigger.addInputListener(this);
        r2DriverTrigger = (WsJoystickAxis) WsInputs.DRIVER_RIGHT_TRIGGER.get();
        r2DriverTrigger.addInputListener(this);
        bButton = (WsJoystickButton) WsInputs.DRIVER_FACE_RIGHT.get();
        bButton.addInputListener(this);
        driverYButton = (WsJoystickButton) WsInputs.DRIVER_FACE_UP.get();
        driverYButton.addInputListener(this);

        r2OperatorTrigger = (WsJoystickAxis) WsInputs.OPERATOR_RIGHT_TRIGGER.get();
        r2OperatorTrigger.addInputListener(this);

        ShuffleboardTab tab = Shuffleboard.getTab("Launcher");
 
        targetLauncherVelocityEntry = tab.add("Target Launcher Velocity", targetLauncherVelocity).getEntry();
        targetHoodAngleEntry = tab.add("Target Hood Angle", targetHoodAngle).getEntry();
        feedingModeAngleEntry = tab.add("Feeding Mode Angle", feedingModeAngle).getEntry();
        feedingModeRpmEntry = tab.add("Feeding Mode RPM", feedingModeRpm).getEntry();
    }

    @Override
    public void initSubsystems() {
        localization = (Localization) Core.getSubsystemManager().getSubsystem(WsSubsystems.LOCALIZATION);
        drive = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        intake = (Intake) Core.getSubsystemManager().getSubsystem(WsSubsystems.INTAKE);
    }

    private double getHoodRotation() {
        return LauncherConstants.HOOD_SCALE * (-hood.getPosition() / LauncherConstants.TOTAL_HOOD_RATIO);
    }

    private void setHoodRotation(double rotation) {
        hood.setPosition(LauncherConstants.TOTAL_HOOD_RATIO * (-rotation / LauncherConstants.HOOD_SCALE));
    }

    private double calculateLauncherSpeed(double distance) {
        if (operatorMode) {
            return targetLauncherVelocity;
        } else {
            return Math.min(373.60706 * distance + 688.26387, 4000);
        }
    }

    private double calculateHoodAngle(double distance) {
        if (operatorMode) {
            return targetHoodAngle;
        } else {
            double curX = localization.getCurrentPose().getX();
            
            if (Core.isBlueAlliance() && curX < LocalizationConstants.BLUE_HUB_X) {
                return 0;
            } else if (!Core.isBlueAlliance() && curX > LocalizationConstants.RED_HUB_X) {
                return 0;
            } else {
                return -2.06313 * distance + 47.06313;
            }
        }
    }

    @Override
    public void update() {
        double distance = localization.getAllianceHubDistance();
        double hoodAngle = isFeedMode ? feedingModeAngle : calculateHoodAngle(targetHoodAngle);
        double newLauncherSpeed = isFeedMode ? feedingModeRpm : calculateLauncherSpeed(distance);

        // preaccell and launcher speed
        if (runLauncher || isFeedMode) {
            if (newLauncherSpeed == 0) {
                launcherMiddle.setSpeed(0);
            } else {
                launcherMiddle.setVelocity(newLauncherSpeed);
            }
            preAccel.setSpeed(preAccelSetSpeed);
            setHoodRotation(hoodAngle);
        } else {
            launcherMiddle.setSpeed(0);
            preAccel.setSpeed(0);
        }

        // operator mode override
        if (operatorMode) {
            newLauncherSpeed = targetLauncherVelocity;
        }

        // vision override mode
        if (drive.getVisionOverride()) {
            newLauncherSpeed = 1850;

            setHoodRotation(0);
            preAccel.setSpeed(1);
            launcherMiddle.setVelocity(newLauncherSpeed);
        }

        // deadband for feed enabling
        if (launcherMiddle.getVelocity() > newLauncherSpeed + 50) {
            shouldFeed = true;
        } else if (launcherMiddle.getVelocity() < newLauncherSpeed - 100 || newLauncherSpeed == 0) {
            shouldFeed = false;
        }

        // enable/disable feed
        if (overrideFeedInput) {
            feed.setSpeed(newFeedSpeed);
        } else if (shouldFeed) { // || drive.getVisionOverride()
            //if (!intake.isDeployed()) intake.deployIntake();
            feed.setSpeed(1);
        } else {
            feed.setSpeed(0);
        }

        if (resetHood) {
            // TODO, run hood to bottom then zero, stop with current spike
        }

        SmartDashboard.putNumber("Hub Distance (m)", distance);
        SmartDashboard.putNumber("Angle Distance (rad)", localization.getAllianceHubAngle());
        SmartDashboard.putNumber("Target Launcher RPM", newLauncherSpeed);
        SmartDashboard.putNumber("Target Hood Angle (Deg)", hoodAngle);

        SmartDashboard.putNumber("Feed Duty Cycle", newFeedSpeed);
        SmartDashboard.putBoolean("Run Launcher", runLauncher);
        SmartDashboard.putBoolean("Is feeding", isFeedMode);

        SmartDashboard.putNumber("Feed Read RPM", feed.getVelocity());
        SmartDashboard.putNumber("Pre-Accel Read RPM", preAccel.getVelocity());
        SmartDashboard.putNumber("Launcher Read RPM", launcherMiddle.getVelocity());
        SmartDashboard.putNumber("Hood Position (Deg)", getHoodRotation());

        targetLauncherVelocity = targetLauncherVelocityEntry.getDouble(targetLauncherVelocity);
        targetHoodAngle = targetHoodAngleEntry.getDouble(targetHoodAngle);
        feedingModeRpm = feedingModeRpmEntry.getDouble(feedingModeRpm);
        feedingModeAngle = feedingModeAngleEntry.getDouble(feedingModeAngle);
    }

    @Override
    public void inputUpdate(Input source) {
        if (source == r2DriverTrigger) {
            boolean hasInput = Math.abs(r2DriverTrigger.getValue()) >= 0.1;

            if (hasInput && !runLauncher) {
                runLauncher = true;
                //drive.setDriveState(SwerveDrive.DriveState.LAUNCH);
            } else if (!hasInput && runLauncher) {
                runLauncher = false;
                drive.setDriveState(SwerveDrive.DriveState.TELEOP);
            }
        } else if (source == l2trigger) {
            if (Math.abs(l2trigger.getValue()) >= 0.5) {
                newFeedSpeed = -1;
                overrideFeedInput = true;
            } else {
                newFeedSpeed = 0;
                overrideFeedInput = false;
            }
        } else if (source == bButton) {
            SparkMaxConfig config = LauncherConstants.hoodConfig();
            if (bButton.getValue()) {
                hoodAngleOnPress = getHoodRotation();
                config.idleMode(IdleMode.kCoast);
            } else if (Math.abs(hoodAngleOnPress - getHoodRotation()) > 3) {
                hood.resetEncoder();
                config.idleMode(IdleMode.kBrake);
            }
            hood.configure(config);
        } else if (source == r2OperatorTrigger) {
            boolean hasInput = Math.abs(r2OperatorTrigger.getValue()) >= 0.1;

            if (hasInput && !runLauncher) {
                runLauncher = true;
                operatorMode = true;
            } else if (!hasInput && runLauncher) {
                runLauncher = false;
                operatorMode = false;
            }

        } else if (source == operatorXButton) { 
            resetHood = operatorXButton.getValue(); // set resetHood to current button value
        } else if (source == driverYButton) {
            isFeedMode = driverYButton.getValue();
        }

    }

    public void startLaunch() {
        runLauncher = true;
    }
    
    public void stopLaunch(){
        runLauncher = false;
    }

    public void constantLaunch(boolean active) {
        runLauncher = active;
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void resetState() {
        newFeedSpeed = 0;
        runLauncher = false;
    }

    @Override
    public String getName() {
        return "Launcher";
    }
}