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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Launcher implements Subsystem {

    // variables
    private WsSpark launcherMiddle;
    private double targetLauncherVelocity = 2500;

    private WsSpark preAccel;

    private WsSpark hood;
    private double targetHoodAngle = 0;
    private double hoodAngleOnPress = 0;

    private WsSpark feed;

    private Localization localization;
    private SwerveDrive drive;
    private Intake intake;

    private WsJoystickAxis driverLeftTrigger;
    private WsJoystickAxis driverRightTrigger;
    private WsJoystickAxis operatorRightTrigger;
    private WsJoystickButton driverBButton;
    private WsJoystickButton driverYButton;
    private WsJoystickButton operatorXButton;

    private GenericEntry targetLauncherVelocityEntry;
    private GenericEntry targetHoodAngleEntry;
    private GenericEntry feedingModeAngleEntry;
    private GenericEntry feedingModeRpmEntry;

    private boolean operatorMode = false;
    private boolean resetHood = false;
    private boolean feedMode = false;

    private boolean wasVisionOverride = false;

    private double feedingModeAngle = 30;
    private double feedingModeRpm = 3400;

    
    private enum LauncherState {
        FORWARD,
        IDLE,
        REVERSE,
        OVERRIDE,
    }
    
    private enum FeedState {
        FORWARD,
        IDLE,
        REVERSE,
        OVERRIDE,
    }
    
    private LauncherState launcherState = LauncherState.IDLE;
    private FeedState feedState = FeedState.IDLE;
    
    @Override
    public void init() {
        launcherMiddle = (WsSpark) WsOutputs.LAUNCHER_MIDDLE.get();

        preAccel = (WsSpark) WsOutputs.PREACCEL.get();
        feed = (WsSpark) WsOutputs.FEEDER.get();
        hood = (WsSpark) WsOutputs.HOOD.get();
        hood.resetEncoder();

        driverLeftTrigger = (WsJoystickAxis) WsInputs.DRIVER_LEFT_TRIGGER.get();
        driverLeftTrigger.addInputListener(this);
        driverRightTrigger = (WsJoystickAxis) WsInputs.DRIVER_RIGHT_TRIGGER.get();
        driverRightTrigger.addInputListener(this);
        driverBButton = (WsJoystickButton) WsInputs.DRIVER_FACE_RIGHT.get();
        driverBButton.addInputListener(this);
        driverYButton = (WsJoystickButton) WsInputs.DRIVER_FACE_UP.get();
        driverYButton.addInputListener(this);

        operatorRightTrigger = (WsJoystickAxis) WsInputs.OPERATOR_RIGHT_TRIGGER.get();
        operatorRightTrigger.addInputListener(this);

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
        if (operatorMode) return targetLauncherVelocity;

        return Math.min(373.60706 * distance + 688.26387, 4000);
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
                return MathUtil.clamp(-2.06313 * distance + 47.06313, 0, LauncherConstants.HOOD_RANGE_DEG);
            }
        }
    }

    @Override
    public void update() {
        double distance = localization.getAllianceHubDistance();
        double newHoodAngle = feedMode ? feedingModeAngle : calculateHoodAngle(distance);
        double newLauncherSpeed = feedMode ? feedingModeRpm : calculateLauncherSpeed(distance);
        boolean isVisionOverride = drive.getVisionOverride();

        // Override button
        if (isVisionOverride) {
            newLauncherSpeed = 1850;
            newHoodAngle = 0;
            launcherState = LauncherState.FORWARD;
        } else if (!isVisionOverride && wasVisionOverride) {
            launcherState = LauncherState.IDLE;
        }

        // Launcher state
        switch (launcherState) {
            case FORWARD:
                if (newLauncherSpeed == 0) {
                    launcherMiddle.setSpeed(0); // prevent jitter from PID loop at 0 rpm
                } else {
                    launcherMiddle.setVelocity(newLauncherSpeed);
                }
                preAccel.setSpeed(1);
                setHoodRotation(newHoodAngle);
                break;
            case IDLE:
                launcherMiddle.setSpeed(0);
                preAccel.setSpeed(0);
                break;
            case REVERSE:
                launcherMiddle.setVelocity(-500);
                preAccel.setSpeed(-1);
                break;
            case OVERRIDE:
                // launcherMiddle.setVelocity();
                // preAccel.setSpeed();
                break;
        }

        // Feed enable/disable logic
        //      deadband for feed enabling
        if (feedState != FeedState.REVERSE && feedState != FeedState.OVERRIDE) {
            if (launcherMiddle.getVelocity() > newLauncherSpeed + 50) {
                feedState = FeedState.FORWARD;
            } else if (launcherMiddle.getVelocity() < newLauncherSpeed - 100 || newLauncherSpeed == 0) {
                feedState = FeedState.IDLE;
            }
        }

        // Feed State 
        switch (feedState) {
            case FORWARD:
                feed.setSpeed(1);
                break;
            case IDLE:
                feed.setSpeed(0);
                break;
            case REVERSE:
                feed.setSpeed(-1);
                break;
            case OVERRIDE:
                //feed.setSpeed()
                break;
        }

        if (resetHood) {
            // TODO, run hood to bottom then zero, stop with current spike
        }
        
        // SmartDashboard/Networktables values - Do not change recorded values below here
        SmartDashboard.putNumber("Hub Distance (m)", distance);
        SmartDashboard.putNumber("Angle Distance (rad)", localization.getAllianceHubAngle());
        SmartDashboard.putNumber("Target Launcher RPM", newLauncherSpeed);
        SmartDashboard.putNumber("Target Hood Angle (Deg)", newHoodAngle);

        SmartDashboard.putBoolean("Is feeding", feedMode);
        
        SmartDashboard.putNumber("Feed Read RPM", feed.getVelocity());
        SmartDashboard.putNumber("Pre-Accel Read RPM", preAccel.getVelocity());
        SmartDashboard.putNumber("Launcher Read RPM", launcherMiddle.getVelocity());
        SmartDashboard.putNumber("Hood Position (Deg)", getHoodRotation());
        
        SmartDashboard.putString("Launcher State", launcherState.toString());

        targetLauncherVelocity = targetLauncherVelocityEntry.getDouble(targetLauncherVelocity);
        targetHoodAngle = targetHoodAngleEntry.getDouble(targetHoodAngle);
        feedingModeRpm = feedingModeRpmEntry.getDouble(feedingModeRpm);
        feedingModeAngle = feedingModeAngleEntry.getDouble(feedingModeAngle);

        wasVisionOverride = isVisionOverride;
    }

    @Override
    public void inputUpdate(Input source) {
        if (source == driverRightTrigger) {
            boolean hasInput = Math.abs(driverRightTrigger.getValue()) >= 0.1;

            if (hasInput && launcherState != LauncherState.FORWARD) {
                launcherState = LauncherState.FORWARD;
            } else if (!hasInput && launcherState == LauncherState.FORWARD) {
                launcherState = LauncherState.IDLE;
            }
        } else if (source == driverLeftTrigger) {
            if (Math.abs(driverLeftTrigger.getValue()) >= 0.5) {
                feedState = FeedState.REVERSE;
            } else {
                feedState = FeedState.IDLE;
            }
        } else if (source == driverBButton) {
            SparkMaxConfig config = LauncherConstants.hoodConfig();
            if (driverBButton.getValue()) {
                hoodAngleOnPress = getHoodRotation();
                config.idleMode(IdleMode.kCoast);
            } else if (Math.abs(hoodAngleOnPress - getHoodRotation()) > 3) {
                hood.resetEncoder();
                config.idleMode(IdleMode.kBrake);
            }
            hood.configure(config);
        } else if (source == operatorRightTrigger) {
            boolean hasInput = Math.abs(operatorRightTrigger.getValue()) >= 0.1;

            if (hasInput && launcherState != LauncherState.FORWARD) {
                launcherState = LauncherState.FORWARD;
                operatorMode = true;
            } else if (!hasInput && launcherState == LauncherState.FORWARD) {
                launcherState = LauncherState.IDLE;
                operatorMode = false;
            }

        } else if (source == operatorXButton) { 
            resetHood = operatorXButton.getValue(); // set resetHood to current button value
        } else if (source == driverYButton) {
            if (driverYButton.getValue()) {
                launcherState = LauncherState.FORWARD;
                feedMode = true;
            } else {
                launcherState = LauncherState.IDLE;
                feedMode = false;
            }

        }

    }

    public void startLaunch() {
        launcherState = LauncherState.FORWARD;
    }
    
    public void stopLaunch() {
        launcherState = LauncherState.IDLE;
    }

    public void setLauncherState(boolean bool) {
        if (bool) {
            launcherState = LauncherState.FORWARD;
        } else {
            launcherState = LauncherState.IDLE;
        }
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void resetState() {
        launcherState = LauncherState.IDLE;
        feedState = FeedState.IDLE;
    }

    @Override
    public String getName() {
        return "Launcher";
    }
}