package org.wildstang.hardware.roborio.outputs.config;

import org.wildstang.framework.hardware.OutputConfig;

/**
 * Contains configurations for Spark Max/Flex motor controllers.
 */
public class WsSparkConfig implements OutputConfig {

    private int m_channel = 0;
    private WsMotorControllers controller;

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
