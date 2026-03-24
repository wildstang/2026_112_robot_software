package org.wildstang.sample.subsystems.intake;

import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsJoystickButton;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.hardware.roborio.outputs.WsTalon;
import org.wildstang.sample.robot.WsInputs;
import org.wildstang.sample.robot.WsOutputs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake implements Subsystem {

    private WsTalon intakeDeploy;
    private WsSpark intakeRoller;
    private WsJoystickButton btnA;
    private WsJoystickButton btnX;
    private final Timer intakeRetractTimer = new Timer();

    private double targetRollerForwardVelocity = 4000;
    private GenericEntry targetRollerForwardVelocityEntry;

    private enum RollerState {
        FORWARD,
        OFF,
        REVERSE, 
        FEED
    }

    private enum IntakeState {
        DEPLOYED,
        RETRACTED,
    }

    private RollerState rollerState = RollerState.OFF;
    private IntakeState intakeState = IntakeState.RETRACTED;

    @Override
    public void init() {
        intakeDeploy = (WsTalon) WsOutputs.INTAKE_DEPLOY.get();
        intakeDeploy.setCurrentLimit(40,40);
        // intakeDeploy.resetEncoder();
        intakeDeploy.initClosedLoop(IntakeConstants.DEPLOY_P, IntakeConstants.DEPLOY_I, IntakeConstants.DEPLOY_D);   // Slot 0
        intakeDeploy.addClosedLoop(IntakeConstants.RETRACT_P, IntakeConstants.RETRACT_I, IntakeConstants.RETRACT_D);  // Slot 1

        intakeRoller = (WsSpark) WsOutputs.INTAKE_SPIN_LEFT.get();

        btnA = (WsJoystickButton) WsInputs.DRIVER_FACE_DOWN.get();
        btnA.addInputListener(this);
        btnX = (WsJoystickButton) WsInputs.DRIVER_FACE_LEFT.get();
        btnX.addInputListener(this);

        intakeRetractTimer.stop();
        intakeRetractTimer.reset();

        ShuffleboardTab tab = Shuffleboard.getTab("Intake");

        targetRollerForwardVelocityEntry = tab.add("Target Intake Roller Forward Velocity", targetRollerForwardVelocity).getEntry();
    }

    @Override
    public void update() {
         if (intakeState == IntakeState.DEPLOYED){
            intakeDeploy.setPosition(IntakeConstants.DEPLOY_ROTATIONS, 0);
        } else  {
            // intakeDeploy.setPosition(IntakeConstants.RETRACT_ROTATIONS, 1);
        }
        
        if (rollerState == RollerState.FEED) {
            // don't change state if in feed mode
        } else if (intakeRetractTimer.isRunning()) { // if timer running
            // stop timer if past n seconds
            if (intakeRetractTimer.hasElapsed(0.75)) { 
                rollerState = RollerState.OFF;
                intakeRetractTimer.stop();
            } else {
                // power rollers for n seconds after retract
                rollerState = RollerState.FORWARD; 
            }
        } else if (intakeState == IntakeState.DEPLOYED) { 
            // Roll rollers while deployed, unless overridden
            if (rollerState != RollerState.REVERSE) rollerState = RollerState.FORWARD;
        } else {
            // Don't run rollers when retracted
            if (rollerState != RollerState.REVERSE) rollerState = RollerState.OFF;
        }

        switch (rollerState) {
            case FORWARD:
                intakeRoller.setVelocity(targetRollerForwardVelocity);
                break;
            case OFF:
                intakeRoller.setSpeed(0);
                break;
            case REVERSE:
                intakeRoller.setSpeed(-1);
                break;
            case FEED:
                intakeRoller.setSpeed(0.5);
                break;
        }

        SmartDashboard.putString("Intake State", intakeState.toString());
        SmartDashboard.putString("Roller State", rollerState.toString());
        SmartDashboard.putNumber("Intake Output Percent", intakeRoller.getOutput());
        
        SmartDashboard.putNumber("Intake Position (Rot)", intakeDeploy.getPosition());
        SmartDashboard.putNumber("Intake Speed (RPM)", intakeRoller.getVelocity());
        SmartDashboard.putNumber("Intake Target Speed (RPM)", targetRollerForwardVelocity);

        targetRollerForwardVelocity = targetRollerForwardVelocityEntry.getDouble(targetRollerForwardVelocity);
    }

    @Override
    public void inputUpdate(Input source) {
        if (source == btnX) {
            if (btnX.getValue()) {
                if (intakeState == IntakeState.DEPLOYED) {
                    intakeState = IntakeState.RETRACTED;
                    intakeRetractTimer.restart();
                } else {
                    intakeState = IntakeState.DEPLOYED;
                }
            }
        }
        if (source == btnA) {
            if (btnA.getValue()) {
                rollerState = RollerState.REVERSE;
            } else {
                if (intakeState == IntakeState.DEPLOYED) {
                    rollerState = RollerState.FORWARD;
                } else {
                    rollerState = RollerState.OFF;
                }
            }
        }
    }

    public void deployIntake() {
        intakeState = IntakeState.DEPLOYED;
    }

    public void removeIntake() {
        intakeState = IntakeState.RETRACTED;
        intakeRetractTimer.restart();
    }

    public void intakeFeed() {
        rollerState = RollerState.FEED;
    }

    public void intakeDisable() {
        rollerState = RollerState.OFF;
    }

    public boolean isDeployed() {
        return intakeState == IntakeState.DEPLOYED;
    }

    @Override
    public void selfTest() {}

    @Override
    public void resetState() {}

    @Override
    public String getName() {
        return "Intake";
    }

    @Override
    public void initSubsystems() {}
}