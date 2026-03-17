package org.wildstang.sample.subsystems.intake;

import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsJoystickButton;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.sample.robot.WsInputs;
import org.wildstang.sample.robot.WsOutputs;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake implements Subsystem {

    private WsSpark intakeDeploy;
    private WsSpark intakeRoller;
    private WsJoystickButton btnA;
    private WsJoystickButton btnX;
    private final Timer intakeRetractTimer = new Timer();

    private enum RollerState {
        FORWARD(1),
        IDLE(0.5), // half speed
        OFF(0),
        REVERSE(-1);

        private final double speed;

        private RollerState(double speed) {
            this.speed = speed;
        } 

        private double getSpeed() {
            return this.speed;
        }
    }

    private enum IntakeState {
        DEPLOYED,
        RETRACTED,
    }

    private RollerState rollerState = RollerState.OFF;
    private IntakeState intakeState = IntakeState.RETRACTED;

    @Override
    public void init() {
        intakeDeploy = (WsSpark) WsOutputs.INTAKE_DEPLOY.get();
        intakeDeploy.resetEncoder();

        intakeRoller = (WsSpark) WsOutputs.INTAKE_SPIN_LEFT.get();
        btnA = (WsJoystickButton) WsInputs.DRIVER_FACE_DOWN.get();
        btnA.addInputListener(this);
        btnX = (WsJoystickButton) WsInputs.DRIVER_FACE_LEFT.get();
        btnX.addInputListener(this);

        intakeRetractTimer.stop();
        intakeRetractTimer.reset();
    }

    @Override
    public void update() {
         if (intakeState == IntakeState.DEPLOYED){
            intakeDeploy.setPosition(IntakeConstants.DEPLOY_ROTATIONS, 0);
        } else  {
            intakeDeploy.setPosition(IntakeConstants.RETRACT_ROTATIONS, 1);
        }
        
        if (intakeRetractTimer.isRunning()) { // if timer running
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

        intakeRoller.setSpeed(rollerState.getSpeed());

        SmartDashboard.putBoolean("Deploy Intake", intakeState == IntakeState.DEPLOYED);
        SmartDashboard.putNumber("Intake Set Speed", rollerState.getSpeed());
        
        SmartDashboard.putNumber("Intake Position (Rot)", intakeDeploy.getPosition());
        SmartDashboard.putNumber("Intake Speed (RPM)", intakeRoller.getVelocity());
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