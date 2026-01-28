package org.wildstang.sample.subsystems;

import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsJoystickButton;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.sample.robot.WsInputs;

import org.wildstang.sample.robot.WsOutputs;

public class Intake implements Subsystem {

    public WsSpark intakeDeploy;
    public WsSpark intakeRoller;
    public WsJoystickButton btnA;
    public WsJoystickButton btnY;
    public WsJoystickButton btnX;
    public double deploySpeed;
    public double rollerSpeed;

    @Override
    public void init() {
        intakeDeploy = (WsSpark) WsOutputs.INTAKE_DEPLOY_LEFT.get();
        intakeRoller = (WsSpark) WsOutputs.INTAKE_SPIN.get();
        btnA = (WsJoystickButton) WsInputs.OPERATOR_FACE_DOWN.get();
        btnA.addInputListener(this);
        btnX = (WsJoystickButton) WsInputs.OPERATOR_FACE_LEFT.get();
        btnX.addInputListener(this);
        btnY = (WsJoystickButton) WsInputs.OPERATOR_FACE_UP.get();
        btnY.addInputListener(this);
    }

    @Override
    public void update() {
        intakeDeploy.setSpeed(deploySpeed);
        intakeRoller.setSpeed(rollerSpeed);
    }

    @Override
    public void inputUpdate(Input source) {
        if (source == btnY || source == btnA) {
            if(btnY.getValue()) {
                deploySpeed = 0.2; //down
            }
            else if(btnA.getValue()) {
                deploySpeed = -1; // up
            }
            else {
                deploySpeed = 0;
            }
        }
        if(source == btnX) {
            if (btnX.getValue()) {
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