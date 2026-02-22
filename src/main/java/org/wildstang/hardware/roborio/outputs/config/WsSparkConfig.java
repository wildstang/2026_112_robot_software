package org.wildstang.hardware.roborio.outputs.config;

import org.wildstang.framework.hardware.OutputConfig;

import com.revrobotics.spark.config.SparkBaseConfig;

/**
 * Contains configurations for Spark Max/Flex motor controllers.
 */
public class WsSparkConfig implements OutputConfig {

    private int m_channel = 0;
    private SparkBaseConfig config;
    private WsMotorControllers controller;
    private boolean isUsingController = false;

    /**
     * Construct the Phoenix config.
     * @param channel Controller CAN constant.
     * @param controller Enumeration representing type of controller.
     */
    public WsSparkConfig(int channel, WsMotorControllers controller) {
        m_channel = channel;
        this.controller = controller;
    }

    /**
     * Construct the Phoenix config.
     * @param channel Controller CAN constant.
     * @param controller Enumeration representing type of controller.
     */
    public WsSparkConfig(int channel, WsMotorControllers controller, SparkBaseConfig config) {
        this(channel, controller, config, false);
    }

    /**
     * Construct the Phoenix config.
     * @param channel Controller CAN constant.
     * @param controller Enumeration representing type of controller.
     * @param isUsingController Whether or not a closed loop controller will be used.
     */
    public WsSparkConfig(int channel, WsMotorControllers controller, SparkBaseConfig config, boolean isUsingController) {
        m_channel = channel;
        this.controller = controller;
        this.config = config;
        this.isUsingController = isUsingController;
    }

    /**
     * Returns the hardware port number.
     * @return The hardware port number.
     */
    public int getChannel() {
        return m_channel;
    }

    /**
     * Returns the motor controller type.
     * @return The hardware motor controller type.
     */
    public WsMotorControllers getType() {
        return controller;
    }

    /**
     * Determines whether the controller has a config.
     * @return True if a config was provided.
     */
    public boolean hasConfig() {
        return config != null;
    }

    /**
     * Returns the motor controller config.
     * @return The provided motor controller config.
     */
    public SparkBaseConfig getConfig() {
        return config;
    }

    /**
     * Returns the using controller state.
     * @return True is a closed loop controller will be used.
     */
    public boolean getUsingController() {
        return isUsingController;
    }

    /**
     * Builds a JSON String describing the Spark Max config.
     * @return Channel number and is talon.
     */
    @Override
    public String toString() {
        StringBuffer buf = new StringBuffer();
        buf.append("{\"channel\": ");
        buf.append(m_channel);
        buf.append(", \"type\": ");
        buf.append(controller.name());
        buf.append("}");
        return buf.toString();
    }

}
