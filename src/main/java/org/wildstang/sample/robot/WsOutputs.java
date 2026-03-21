package org.wildstang.sample.robot;

// expand this and edit if trouble with Ws
import org.wildstang.framework.core.Core;
import org.wildstang.framework.core.Outputs;
import org.wildstang.framework.hardware.OutputConfig;
import org.wildstang.framework.io.outputs.Output;
import org.wildstang.hardware.roborio.outputs.config.WsMotorControllers;
import org.wildstang.hardware.roborio.outputs.config.WsSparkConfig;
import org.wildstang.hardware.roborio.outputs.config.WsTalonConfig;
import org.wildstang.sample.subsystems.intake.IntakeConstants;
import org.wildstang.sample.subsystems.launcher.LauncherConstants;
import org.wildstang.sample.subsystems.swerve.ModuleConstants;

/**
 * Output mappings are stored here.
 * Below each Motor, PWM, Digital Output, Solenoid, and Relay is enumerated with their appropriated IDs.
 * The enumeration includes a name, output type, and output config object.
 */
public enum WsOutputs implements Outputs {

    // ---------------------------------
    // Drive Motors
    // ---------------------------------
    DRIVE1("Module 1 Drive Motor", new WsSparkConfig(CANConstants.DRIVE1, WsMotorControllers.SPARK_FLEX_BRUSHLESS, ModuleConstants.driveConfig(), true)),
    ANGLE1("Module 1 Angle Motor", new WsSparkConfig(CANConstants.ANGLE1, WsMotorControllers.SPARK_MAX_BRUSHLESS, ModuleConstants.angleConfig(), true)),
    DRIVE2("Module 2 Drive Motor", new WsSparkConfig(CANConstants.DRIVE2, WsMotorControllers.SPARK_FLEX_BRUSHLESS, ModuleConstants.driveConfig(), true)),
    ANGLE2("Module 2 Angle Motor", new WsSparkConfig(CANConstants.ANGLE2, WsMotorControllers.SPARK_MAX_BRUSHLESS, ModuleConstants.angleConfig(), true)),
    DRIVE3("Module 3 Drive Motor", new WsSparkConfig(CANConstants.DRIVE3, WsMotorControllers.SPARK_FLEX_BRUSHLESS, ModuleConstants.driveConfig(), true)),
    ANGLE3("Module 3 Angle Motor", new WsSparkConfig(CANConstants.ANGLE3, WsMotorControllers.SPARK_MAX_BRUSHLESS, ModuleConstants.angleConfig(), true)),
    DRIVE4("Module 4 Drive Motor", new WsSparkConfig(CANConstants.DRIVE4, WsMotorControllers.SPARK_FLEX_BRUSHLESS, ModuleConstants.driveConfig(), true)),
    ANGLE4("Module 4 Angle Motor", new WsSparkConfig(CANConstants.ANGLE4, WsMotorControllers.SPARK_MAX_BRUSHLESS, ModuleConstants.angleConfig(), true)),
    
    // ---------------------------------
    // Other Motors
    // ---------------------------------

    INTAKE_DEPLOY("Intake Deploy Motor", new WsTalonConfig(CANConstants.INTAKE_DEPLOY, WsMotorControllers.TALON_FX)),
    INTAKE_SPIN_LEFT("Left Intake Spin Motor", new WsSparkConfig(CANConstants.INTAKE_SPIN_LEFT, WsMotorControllers.SPARK_FLEX_BRUSHLESS, IntakeConstants.leftSpinConfig())),
    INTAKE_SPIN_RIGHT("Right Intake Spin Motor", new WsSparkConfig(CANConstants.INTAKE_SPIN_RIGHT, WsMotorControllers.SPARK_FLEX_BRUSHLESS, IntakeConstants.rightSpinConfig())),

    FEEDER("Feeder Motor", new WsSparkConfig(CANConstants.FEEDER, WsMotorControllers.SPARK_MAX_BRUSHLESS, LauncherConstants.feedConfig())),
    PREACCEL("Pre-Accel Motor", new WsSparkConfig(CANConstants.PREACCEL, WsMotorControllers.SPARK_MAX_BRUSHLESS, LauncherConstants.preAccelConfig())),
    LAUNCHER_MIDDLE("Launcher Middle Motor", new WsSparkConfig(CANConstants.LAUNCHER_MIDDLE, WsMotorControllers.SPARK_FLEX_BRUSHLESS, LauncherConstants.middleConfig(), true)),
    LAUNCHER_LEFT("Launcher Left Motor", new WsSparkConfig(CANConstants.LAUNCHER_LEFT, WsMotorControllers.SPARK_FLEX_BRUSHLESS, LauncherConstants.leftConfig(), true)),
    LAUNCHER_RIGHT("Launcher Right Motor", new WsSparkConfig(CANConstants.LAUNCHER_RIGHT, WsMotorControllers.SPARK_FLEX_BRUSHLESS, LauncherConstants.rightConfig(), true)),
    HOOD("Hood Motor", new WsSparkConfig(CANConstants.HOOD, WsMotorControllers.SPARK_MAX_BRUSHLESS, LauncherConstants.hoodConfig(), true)),

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
