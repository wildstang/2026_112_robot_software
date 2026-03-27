package org.wildstang.sample.subsystems.intake;

import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsDPadButton;
import org.wildstang.hardware.roborio.inputs.WsJoystickButton;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.hardware.roborio.outputs.WsTalon;
import org.wildstang.sample.robot.WsInputs;
import org.wildstang.sample.robot.WsOutputs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake implements Subsystem {

    private WsTalon intakeDeploy;
    private WsSpark intakeRoller;
    private WsJoystickButton btnA;
    private WsJoystickButton btnX;
    private WsDPadButton dPadUp;

    private double targetRollerForwardVelocity = 4000;
    private GenericEntry targetRollerForwardVelocityEntry;

    private enum RollerState {
        FORWARD,
        OFF,
        REVERSE, 
        HALF_FORWARD
    }

    private enum IntakeState {
        DEPLOY,
        RETRACT,
        INGEST
    }

    private RollerState rollerState = RollerState.OFF;
    private IntakeState intakeState = IntakeState.RETRACT;

    @Override
    public void init() {
        intakeDeploy = (WsTalon) WsOutputs.INTAKE_DEPLOY.get();
        intakeDeploy.setCurrentLimit(40,40);
        intakeDeploy.initClosedLoop(IntakeConstants.DEPLOY_P, IntakeConstants.DEPLOY_I, IntakeConstants.DEPLOY_D, 0);   // Slot 0
        intakeDeploy.initClosedLoop(IntakeConstants.RETRACT_P, IntakeConstants.RETRACT_I, IntakeConstants.RETRACT_D, 1);  // Slot 1
        intakeDeploy.initClosedLoop(IntakeConstants.INGEST_P, IntakeConstants.INGEST_I, IntakeConstants.INGEST_D, 2); // Slot 2

        intakeRoller = (WsSpark) WsOutputs.INTAKE_SPIN_LEFT.get();

        btnA = (WsJoystickButton) WsInputs.DRIVER_FACE_DOWN.get();
        btnA.addInputListener(this);
        btnX = (WsJoystickButton) WsInputs.DRIVER_FACE_LEFT.get();
        btnX.addInputListener(this);
        dPadUp = (WsDPadButton) WsInputs.DRIVER_DPAD_UP.get();
        dPadUp.addInputListener(this);

        ShuffleboardTab tab = Shuffleboard.getTab("Intake");

        targetRollerForwardVelocityEntry = tab.add("Target Intake Roller Forward Velocity", targetRollerForwardVelocity).getEntry();
    }

    @Override
    public void update() {

        switch (intakeState) {
            case DEPLOY:
                intakeDeploy.setPosition(IntakeConstants.DEPLOY_ROTATIONS, 0);
                break;
            case RETRACT:
                intakeDeploy.setPosition(IntakeConstants.RETRACT_ROTATIONS, 1);
                break;
            case INGEST:
                // intakeDeploy.setPosition(IntakeConstants.RETRACT_ROTATIONS, 2);
                break;
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
            case HALF_FORWARD:
                intakeRoller.setVelocity(targetRollerForwardVelocity / 2);
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
                if (intakeState == IntakeState.DEPLOY) {
                    if (rollerState == RollerState.FORWARD) {
                        setRollerState(RollerState.OFF);
                    } else {
                        setRollerState(RollerState.FORWARD);
                    }
                } else {
                    setIntakeState(IntakeState.DEPLOY);
                    
                }
            }
        } else if (source == btnA) {
            if (btnA.getValue()) {
                setRollerState(RollerState.REVERSE);
            } else {
                if (intakeState == IntakeState.DEPLOY) {
                    setRollerState(RollerState.FORWARD);
                } else {
                    setRollerState(RollerState.OFF);
                }
            }
        } else if (source == dPadUp) {
            if (dPadUp.getValue()) {
                setIntakeState(IntakeState.RETRACT);
            }
        }
    }

    public void setIntakeState(IntakeState newIntakeState) {
        switch (newIntakeState) {
            case DEPLOY:
                intakeState = IntakeState.DEPLOY;
                rollerState = RollerState.FORWARD;
                break;
            case RETRACT:
                intakeState = IntakeState.RETRACT;
                rollerState = RollerState.OFF;
                break;
            case INGEST:
                intakeState = IntakeState.INGEST;
                rollerState = RollerState.HALF_FORWARD;
                break;
        }
    }

    public void setRollerState(RollerState newRollerState) {
        rollerState = newRollerState;
    }

    public void deployIntake() {
        setIntakeState(IntakeState.DEPLOY);
        setRollerState(RollerState.FORWARD);
    }

    public void retractIntake() {
        setIntakeState(IntakeState.RETRACT);
    }

    public void setIngestMode(boolean newValue) {
        if (newValue) {
            setIntakeState(IntakeState.INGEST);
        } else if (intakeState == IntakeState.INGEST) {
            setIntakeState(IntakeState.DEPLOY);
            setRollerState(RollerState.FORWARD);
        }
    }

    public void rollersDisable() {
        setRollerState(RollerState.OFF);
    }

    public boolean isDeployed() {
        return intakeState == IntakeState.DEPLOY;
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