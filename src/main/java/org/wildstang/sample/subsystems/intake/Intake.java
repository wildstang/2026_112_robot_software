package org.wildstang.sample.subsystems.intake;

import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsJoystickButton;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.sample.robot.WsInputs;

import org.wildstang.sample.robot.WsOutputs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake implements Subsystem {

    public WsSpark intakeDeploy;
    public WsSpark intakeRoller;
    public WsJoystickButton btnA;
    public WsJoystickButton btnY;
    public WsJoystickButton btnX;
    public boolean deployed = false;
    public double rollerSpeed;

    @Override
    public void init() {
        intakeDeploy = (WsSpark) WsOutputs.INTAKE_DEPLOY_LEFT.get();
        intakeDeploy.resetEncoder();

        intakeRoller = (WsSpark) WsOutputs.INTAKE_SPIN.get();
        btnA = (WsJoystickButton) WsInputs.DRIVER_FACE_DOWN.get();
        btnA.addInputListener(this);
        btnX = (WsJoystickButton) WsInputs.DRIVER_FACE_LEFT.get();
        btnX.addInputListener(this);
    }

    @Override
    public void update() {
        if (deployed) {
            intakeDeploy.setPosition(IntakeConstants.DEPLOY_ROTATIONS, 0);
        }
        else {
            intakeDeploy.setPosition(IntakeConstants.RETRACT_ROTATIONS, 1);
        }
        intakeRoller.setSpeed(rollerSpeed);

        SmartDashboard.putBoolean("Deploy Intake", deployed);
        SmartDashboard.putNumber("Intake Set Speed", rollerSpeed);
        
        SmartDashboard.putNumber("Intake Position (Rot)", intakeDeploy.getPosition());
        SmartDashboard.putNumber("Intake Speed (RPM)", intakeRoller.getVelocity());
    }

    @Override
    public void inputUpdate(Input source) {
        if (source == btnX) {
            if(btnX.getValue()) {
                deployed = !deployed;
            }
        }
        if(source == btnA) {
            if (btnA.getValue()) {
                rollerSpeed = 1;
            }
            else {
                rollerSpeed = 0;
            }
        }
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