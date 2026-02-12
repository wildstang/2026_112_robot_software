package org.wildstang.sample.robot;

// expand this and edit if trouble with Ws
import org.wildstang.framework.core.Core;
import org.wildstang.framework.core.Outputs;
import org.wildstang.framework.hardware.OutputConfig;
import org.wildstang.framework.io.outputs.Output;
import org.wildstang.hardware.roborio.outputs.config.WsMotorControllers;
import org.wildstang.hardware.roborio.outputs.config.WsSparkConfig;
import org.wildstang.hardware.roborio.outputs.config.WsSparkFollowerConfig;
import org.wildstang.hardware.roborio.outputs.config.WsTalonConfig;

/**
 * Output mappings are stored here.
 * Below each Motor, PWM, Digital Output, Solenoid, and Relay is enumerated with their appropriated IDs.
 * The enumeration includes a name, output type, and output config object.
 */
public enum WsOutputs implements Outputs {

    // ---------------------------------
    // Drive Motors
    // ---------------------------------
    DRIVE1("Module 1 Drive Motor", new WsSparkConfig(CANConstants.DRIVE1, WsMotorControllers.SPARK_FLEX_BRUSHLESS)),
    ANGLE1("Module 1 Angle Motor", new WsSparkConfig(CANConstants.ANGLE1, WsMotorControllers.SPARK_MAX_BRUSHLESS)),
    DRIVE2("Module 2 Drive Motor", new WsSparkConfig(CANConstants.DRIVE2, WsMotorControllers.SPARK_FLEX_BRUSHLESS)),
    ANGLE2("Module 2 Angle Motor", new WsSparkConfig(CANConstants.ANGLE2, WsMotorControllers.SPARK_MAX_BRUSHLESS)),
    DRIVE3("Module 3 Drive Motor", new WsSparkConfig(CANConstants.DRIVE3, WsMotorControllers.SPARK_FLEX_BRUSHLESS)),
    ANGLE3("Module 3 Angle Motor", new WsSparkConfig(CANConstants.ANGLE3, WsMotorControllers.SPARK_MAX_BRUSHLESS)),
    DRIVE4("Module 4 Drive Motor", new WsSparkConfig(CANConstants.DRIVE4, WsMotorControllers.SPARK_FLEX_BRUSHLESS)),
    ANGLE4("Module 4 Angle Motor", new WsSparkConfig(CANConstants.ANGLE4, WsMotorControllers.SPARK_MAX_BRUSHLESS)),
    
    // ---------------------------------
    // Other Motors
    // ---------------------------------

    CLIMB_LEFT("Left Climb Motor", new WsSparkConfig(CANConstants.CLIMB_LEFT, WsMotorControllers.SPARK_FLEX_BRUSHLESS)),
    CLIMB_RIGHT("Right Climb Motor", new WsSparkFollowerConfig("Left Climb Motor", CANConstants.CLIMB_RIGHT, WsMotorControllers.SPARK_FLEX_BRUSHLESS, true)),
    
    INTAKE_DEPLOY_LEFT("Left Intake Deploy Motor", new WsTalonConfig(CANConstants.INTAKE_DEPLOY_LEFT)),
    INTAKE_SPIN("Intake Spin Motor", new WsSparkConfig(CANConstants.INTAKE_SPIN, WsMotorControllers.SPARK_FLEX_BRUSHLESS)),
    INTAKE_DEPLOY_RIGHT("Right Intake Deploy Motor", new WsTalonConfig(CANConstants.INTAKE_DEPLOY_RIGHT)),

    FEEDER("Feeder Motor", new WsSparkConfig(CANConstants.FEEDER, WsMotorControllers.SPARK_MAX_BRUSHLESS)),
    PREACCEL("Pre-Accel Motor", new WsSparkConfig(CANConstants.PREACCEL, WsMotorControllers.SPARK_MAX_BRUSHLESS)),
    LAUNCHER_MIDDLE("Launcher Middle Motor", new WsSparkConfig(CANConstants.LAUNCHER_MIDDLE, WsMotorControllers.SPARK_FLEX_BRUSHLESS)),
    LAUNCHER_LEFT("Launcher Left Motor", new WsSparkFollowerConfig("Launcher Middle Motor", CANConstants.LAUNCHER_LEFT, WsMotorControllers.SPARK_FLEX_BRUSHLESS, false)),
    LAUNCHER_RIGHT("Launcher Right Motor", new WsSparkFollowerConfig("Launcher Middle Motor", CANConstants.LAUNCHER_RIGHT, WsMotorControllers.SPARK_FLEX_BRUSHLESS,true)),
    HOOD("Hood Motor", new WsSparkConfig(CANConstants.HOOD, WsMotorControllers.SPARK_MAX_BRUSHLESS)),

    ; // end of enum

    /**
     * Do not modify below code, provides template for enumerations.
     * We would like to have a super class for this structure, however,
     * Java does not support enums extending classes.
     */

    private String m_name;
    private OutputConfig m_config;
    private boolean m_enabled;

    /**
     * Initialize a new Output.
     * @param p_name Name, must match that in class to prevent errors.
     * @param p_config Corresponding configuration for OutputType.
     */
    WsOutputs(String p_name, OutputConfig p_config) {
        this(p_name, p_config, true);
    }

    /**
     * Initialize a new Output.
     * @param p_name Name, must match that in class to prevent errors.
     * @param p_config Corresponding configuration for OutputType.
     */
    WsOutputs(String p_name, OutputConfig p_config, boolean p_enabled) {
        m_name = p_name;
        m_config = p_config;
        m_enabled = p_enabled;
    }

    /**
     * Returns the name mapped to the Output.
     * @return Name mapped to the Output.
     */
    public String getName() {
        return m_name;
    }

    /**
     * Returns the config of Output for the enumeration.
     * @return OutputConfig of enumeration.
     */
    public OutputConfig getConfig() {
        return m_config;
    }

    /**
     * Returns whether the Output is enabled.
     * @return Output's enabled.
     */
    @Override
    public boolean isEnabled() {
        return m_enabled;
    }

    /**
     * Returns the actual Output object from the OutputManager
     * @return The corresponding output.
     */
    public Output get() {
        return Core.getOutputManager().getOutput(this);
    }
}