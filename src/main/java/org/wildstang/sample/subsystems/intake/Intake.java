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
    private WsJoystickButton btnY;
    private WsJoystickButton btnX;
    private boolean deployed = false;
    private double rollerSpeed;
    private Timer intakeRetractTimer = new Timer();
    private boolean wasDeployed = false;
    private double iterations = 0;


    @Override
    public void init() {
        intakeDeploy = (WsSpark) WsOutputs.INTAKE_DEPLOY_LEFT.get();
        intakeDeploy.resetEncoder();

        intakeRoller = (WsSpark) WsOutputs.INTAKE_SPIN.get();
        btnA = (WsJoystickButton) WsInputs.DRIVER_FACE_DOWN.get();
        btnA.addInputListener(this);
        btnX = (WsJoystickButton) WsInputs.DRIVER_FACE_LEFT.get();
        btnX.addInputListener(this);

        intakeRetractTimer.stop();
        intakeRetractTimer.reset();
    }

    @Override
    public void update() {
         if (deployed){
            intakeDeploy.setPosition(IntakeConstants.DEPLOY_ROTATIONS, 0);
        } else  {
            intakeDeploy.setPosition(IntakeConstants.RETRACT_ROTATIONS, 1);
        }
        
        if (wasDeployed && !deployed) {
            intakeRetractTimer.restart();
        } else if (intakeRetractTimer.isRunning()) {
            if (intakeRetractTimer.hasElapsed(0.75)) {
                rollerSpeed = 0;
                intakeRetractTimer.stop();
            } else {
                rollerSpeed = 1;
            }
        } else {
            intakeRetractTimer.stop();
        }
        
        if (deployed && !btnA.getValue()) rollerSpeed = 0.5;

        intakeRoller.setSpeed(rollerSpeed);

        SmartDashboard.putBoolean("Deploy Intake", deployed);
        SmartDashboard.putNumber("Intake Set Speed", rollerSpeed);
        
        SmartDashboard.putNumber("Intake Position (Rot)", intakeDeploy.getPosition());
        SmartDashboard.putNumber("Intake Speed (RPM)", intakeRoller.getVelocity());
        wasDeployed = deployed;
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
                deployed = true;
            }
        }
    }

    public void deployIntake() {
        deployed = true;
        rollerSpeed = 1;
    }

    public void removeIntake() {
        deployed = false;
        rollerSpeed = 0;
    }

    public boolean isDeployed() {
        return deployed;
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