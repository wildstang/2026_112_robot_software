package org.wildstang.hardware.roborio.outputs;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj.RobotBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.wildstang.framework.logger.Log;
import org.wildstang.hardware.roborio.outputs.config.WsMotorControllers;

/**
 * Controls a Spark Max/Flex motor controller.
 * 
 * @author Liam
 */
public class WsSpark extends WsMotorController {

    SparkBase motor;

    boolean isChanged;
    ControlType controlType;

    double arbitraryFF;
    boolean isUsingController;
    ClosedLoopSlot positionSlotID;

    /**
     * Constructs the motor controller from a config.
     * 
     * @param name       Descriptive name of the controller.
     * @param channel    Motor controller CAN constant.
     * @param controller Enumeration representing type of controller.
     * @param isUsingController Whether a closed loop controller is being used.
     */
    public WsSpark(String name, int channel, WsMotorControllers controller, boolean isUsingController) {
        super(name);

        boolean brushless = controller == WsMotorControllers.SPARK_MAX_BRUSHLESS
                || controller == WsMotorControllers.SPARK_FLEX_BRUSHLESS;
        switch (controller) {
            case SPARK_MAX_BRUSHED:
            case SPARK_MAX_BRUSHLESS:
                motor = new SparkMax(channel, brushless ? MotorType.kBrushless : MotorType.kBrushed);
                break;
            case SPARK_FLEX_BRUSHED:
            case SPARK_FLEX_BRUSHLESS:
                motor = new SparkFlex(channel, brushless ? MotorType.kBrushless : MotorType.kBrushed);
                break;
            default:
                Log.error("Invalid motor controller for WsSpark!");
                return;
        }

        isChanged = true;
        controlType = ControlType.kDutyCycle;

        arbitraryFF = 0;
        this.isUsingController = isUsingController;
        positionSlotID = ClosedLoopSlot.kSlot0;
    }

    /**
     * Send a given config to the controller, but do not burn it in permanently.
     * @param config Corresponding SparkMaxConfig or SparkFlexConfig.
     */
    public void configure(SparkBaseConfig config) {
        configure(config, false);
    }

    /**
     * Send a given config to the controller.
     * @param config Corresponding SparkMaxConfig or SparkFlexConfig.
     * @param persist Whether to burn the config permanently to the controller.
     */
    public void configure(SparkBaseConfig config, boolean persist) {
        if (!RobotBase.isReal())
            return;

        motor.configureAsync(config, ResetMode.kResetSafeParameters,
                persist ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    }

    /**
     * Returns the raw motor controller Object.
     * 
     * @return SparkBase Object.
     */
    public SparkBase getController() {
        return motor;
    }

    /**
     * Does nothing, use configure() to send a config with brake mode active.
     */
    public void setBrake() {
        throw new UnsupportedOperationException("setBrake cannot be used for WsSpark");
    }

    /**
     * Does nothing, use configure() to send a config with coast mode active.
     */
    public void setCoast() {
        throw new UnsupportedOperationException("setCoast cannot be used for WsSpark");}

    /**
     * Returns the quadrature velocity from an encoder.
     * 
     * @return Current velocity.
     */
    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    /**
     * Returns the quadrature position from an encoder.
     * 
     * @return Current position.
     */
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    /**
     * Resets the position of an encoder.
     */
    public void resetEncoder() {
        motor.getEncoder().setPosition(0.0);
    }

    /**
     * Returns the current motor output percent.
     * 
     * @return Current motor output as a percent.
     */
    public double getOutput() {
        return motor.getAppliedOutput();
    }

    /**
     * Returns the temperature of the motor.
     */
    @Override
    public double getTemperature() {
        return motor.getMotorTemperature();
    }

    /**
     * Sets motor control.
     */
    @Override
    public void sendDataToOutput() {
        if (isChanged) {
            if (!isUsingController) {
                motor.set(getValue());
            } else if (controlType == ControlType.kPosition) {
                motor.getClosedLoopController().setSetpoint(super.getValue(), ControlType.kPosition,
                        positionSlotID, arbitraryFF, SparkClosedLoopController.ArbFFUnits.kPercentOut);
            } else {
                motor.getClosedLoopController().setSetpoint(super.getValue(), controlType);
            }
        }
    }

    /**
     * Wraps setValue().
     * 
     * @param value New motor percent speed, from -1.0 to 1.0.
     */
    @Override
    public void setSpeed(double value) {
        isChanged = !(controlType == ControlType.kDutyCycle && super.getValue() == value);
        controlType = ControlType.kDutyCycle;
        super.setSpeed(value);
    }

    /**
     * Sets the motor to track the given position
     * 
     * @param target the encoder target value to track to
     */
    public void setPosition(double target) {
        isChanged = !(super.getValue() == target && controlType == ControlType.kPosition);
        positionSlotID = ClosedLoopSlot.kSlot0;
        arbitraryFF = 0;
        super.setValue(target);
        controlType = ControlType.kPosition;
    }

    /**
     * Sets the motor to track the given position with a specific PID constants
     * 
     * @param target the encoder target to track to
     * @param slotID the ID slot of the motor controller to use
     */
    public void setPosition(double target, int slotID) {
        isChanged = !(super.getValue() == target && controlType == ControlType.kPosition);
        positionSlotID = slotID == 0 ? ClosedLoopSlot.kSlot0
                       : slotID == 1 ? ClosedLoopSlot.kSlot1
                       : slotID == 2 ? ClosedLoopSlot.kSlot2
                                     : ClosedLoopSlot.kSlot3;
        arbitraryFF = 0;
        super.setValue(target);
        controlType = ControlType.kPosition;
    }

    /**
     * Sets the motor to track the given position with a specific PIDF constants
     * 
     * @param target the encoder target to track to
     * @param slotID the ID slot of the controller to use
     */
    public void setPosition(double target, int slotID, double feedForward) {
        isChanged = !(super.getValue() == target && controlType == ControlType.kPosition);
        super.setValue(target);

        positionSlotID = slotID == 0 ? ClosedLoopSlot.kSlot0
                       : slotID == 1 ? ClosedLoopSlot.kSlot1
                       : slotID == 2 ? ClosedLoopSlot.kSlot2
                                     : ClosedLoopSlot.kSlot3;
        arbitraryFF = feedForward;
        controlType = ControlType.kPosition;
    }

    /**
     * Sets the motor to track the given velocity
     * 
     * @param target the encoder target value velocity to track to
     */
    public void setVelocity(double target) {
        isChanged = !(super.getValue() == target && controlType == ControlType.kVelocity);
        super.setValue(target);
        controlType = ControlType.kVelocity;
    }

    /**
     * Does nothing, config values only affects start state.
     */
    public void notifyConfigChange() {}

}
