package org.wildstang.sample.subsystems.launcher;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsDigitalInput;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Launcher implements Subsystem {

    // variables
    private WsDigitalInput beamBreakSensor;
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
    private GenericEntry visionOverrideLauncherSpeedEntry;
    private GenericEntry unJamForwardDurationEntry;
    private GenericEntry unJamReverseDurationEntry;

    private boolean operatorMode = false;
    private boolean resetHood = false;
    private boolean feedMode = false;
    private boolean wasVisionOverride = false;

    private double feedingModeAngle = 30;
    private double feedingModeRpm = 3400;
    private double visionOverrideLauncherSpeed = 1850;
    private double unJamForwardDuration = 2;
    private double unJamReverseDuration = 0;

    private final Timer unJamTimer = new Timer();

    
    private enum LauncherState {
        FORWARD,
        IDLE,
        REVERSE,
        VISION_OVERRIDE,
    }
    
    private enum FeedState {
        FORWARD,
        IDLE,
        REVERSE,
        MANUAL_OVERRIDE, // override button
        AUTO, // when holding down launcher
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
        operatorXButton = (WsJoystickButton) WsInputs.OPERATOR_FACE_LEFT.get();
        operatorXButton.addInputListener(this);


        ShuffleboardTab tab = Shuffleboard.getTab("Launcher");
 
        targetLauncherVelocityEntry = tab.add("Target Launcher Velocity", targetLauncherVelocity).getEntry();
        targetHoodAngleEntry = tab.add("Target Hood Angle", targetHoodAngle).getEntry();
        feedingModeAngleEntry = tab.add("Feeding Mode Angle", feedingModeAngle).getEntry();
        feedingModeRpmEntry = tab.add("Feeding Mode RPM", feedingModeRpm).getEntry();
        visionOverrideLauncherSpeedEntry = tab.add("Vision Override Launcher Speed", visionOverrideLauncherSpeed).getEntry();
        unJamForwardDurationEntry = tab.add("Unjam Forward Duration", unJamForwardDuration).getEntry();
        unJamReverseDurationEntry = tab.add("Unjam Reverse Duration", unJamReverseDuration).getEntry();

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
        

        return Math.min(384.24323 * distance + 647.1549, 4000);
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
            launcherState = LauncherState.VISION_OVERRIDE;
            feedState = FeedState.AUTO;
            newLauncherSpeed = visionOverrideLauncherSpeed;
        } else if (!isVisionOverride && wasVisionOverride) {
            launcherState = LauncherState.IDLE;
            feedState = FeedState.IDLE;
        }
        
        intake.setIngestMode(false);
        // Launcher state
        switch (launcherState) {
            case FORWARD:
                if (newLauncherSpeed <= 15) {
                    launcherMiddle.setSpeed(0); // prevent jitter from PID loop at ~0 rpm
                } else {
                    launcherMiddle.setVelocity(newLauncherSpeed);
                }
                preAccel.setSpeed(1);
                setHoodRotation(newHoodAngle);
                drive.setToLaunch();

                intake.setIngestMode(true);
                break;
            case IDLE:
                launcherMiddle.setSpeed(0);
                preAccel.setSpeed(0);
                break;
            case REVERSE:
                launcherMiddle.setVelocity(-500);
                preAccel.setSpeed(-1);
                break;
            case VISION_OVERRIDE:
                launcherMiddle.setVelocity(newLauncherSpeed);
                preAccel.setSpeed(1);
                setHoodRotation(0);
                break;
        }
        if (feedState == FeedState.AUTO) {
            if (!unJamTimer.isRunning()) unJamTimer.restart();
        } else {
            unJamTimer.stop();
            unJamTimer.reset();
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
            case MANUAL_OVERRIDE:
                //feed.setSpeed()
                break;
            case AUTO:
                if (unJamTimer.get() <= unJamForwardDuration) {
                    if (launcherMiddle.getVelocity() >= newLauncherSpeed) {
                    feed.setSpeed(1);
                } else if (launcherMiddle.getVelocity() < newLauncherSpeed - 150 || newLauncherSpeed == 0) {
                    feed.setSpeed(0);
                    }
                } else if (unJamTimer.get() <= (unJamForwardDuration + unJamReverseDuration)){
                    feed.setSpeed(-1);
                } else {
                    unJamTimer.stop();
                    unJamTimer.reset();
                }
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
        SmartDashboard.putBoolean("Is vision override mode", isVisionOverride);
        
        SmartDashboard.putNumber("Feed Read RPM", feed.getVelocity());
        SmartDashboard.putNumber("Pre-Accel Read RPM", preAccel.getVelocity());
        SmartDashboard.putNumber("Launcher Read RPM", launcherMiddle.getVelocity());
        SmartDashboard.putNumber("Hood Position (Deg)", getHoodRotation());
        
        SmartDashboard.putString("Launcher State", launcherState.toString());
        SmartDashboard.putString("Feed State", feedState.toString());

        targetLauncherVelocity = targetLauncherVelocityEntry.getDouble(targetLauncherVelocity);
        targetHoodAngle = targetHoodAngleEntry.getDouble(targetHoodAngle);
        feedingModeRpm = feedingModeRpmEntry.getDouble(feedingModeRpm);
        feedingModeAngle = feedingModeAngleEntry.getDouble(feedingModeAngle);
        visionOverrideLauncherSpeed = visionOverrideLauncherSpeedEntry.getDouble(visionOverrideLauncherSpeed);
        unJamForwardDuration = unJamForwardDurationEntry.getDouble(unJamForwardDuration);
        unJamReverseDuration = unJamReverseDurationEntry.getDouble(unJamReverseDuration);

        wasVisionOverride = isVisionOverride;
    }

    @Override
    public void inputUpdate(Input source) {
        if (source == driverRightTrigger) {
            boolean hasInput = Math.abs(driverRightTrigger.getValue()) >= 0.1;

            if (hasInput && launcherState != LauncherState.FORWARD) {
                launcherState = LauncherState.FORWARD;
                feedState = FeedState.AUTO;
            } else if (!hasInput && launcherState == LauncherState.FORWARD) {
                launcherState = LauncherState.IDLE;
                feedState = FeedState.IDLE;
                drive.setToTeleop();
                intake.rollersDisable();
            }
        } else if (source == driverLeftTrigger) {
            if (Math.abs(driverLeftTrigger.getValue()) >= 0.5) {
                feedState = FeedState.REVERSE;
            } else {
                if (launcherState != LauncherState.IDLE) {
                    feedState = FeedState.AUTO;
                } else {
                    feedState = FeedState.IDLE;
                }
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
                feedState = FeedState.AUTO;
                operatorMode = true;
            } else if (!hasInput && launcherState == LauncherState.FORWARD) {
                launcherState = LauncherState.IDLE;
                feedState = FeedState.IDLE;
                operatorMode = false;
            }

        } else if (source == operatorXButton) { 
            resetHood = operatorXButton.getValue(); // set resetHood to current button value
        } else if (source == driverYButton) {
            if (driverYButton.getValue()) {
                launcherState = LauncherState.FORWARD;
                feedState = FeedState.AUTO;
                feedMode = true;
            } else {
                launcherState = LauncherState.IDLE;
                feedState = FeedState.IDLE;
                feedMode = false;
            }

        }

    }

    public void startLaunch() {
        launcherState = LauncherState.FORWARD;
        feedState = FeedState.AUTO;
    }
    
    public void stopLaunch() {
        launcherState = LauncherState.IDLE;
        feedState = FeedState.IDLE;
    }

    public void setLauncherState(boolean bool) {
        if (bool) {
            launcherState = LauncherState.FORWARD;
            feedState = FeedState.AUTO;
        } else {
            launcherState = LauncherState.IDLE;
            feedState = FeedState.IDLE;
        }
    }
    
    public boolean isRunning() {
        return launcherState == LauncherState.FORWARD;
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
