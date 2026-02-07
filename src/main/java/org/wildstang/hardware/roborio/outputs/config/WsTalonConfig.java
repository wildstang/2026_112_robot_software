package org.wildstang.hardware.roborio.outputs.config;

import org.wildstang.framework.hardware.OutputConfig;

/**
 * Contains configurations for Phoenix Talon and Victor motor controllers.
 */
public class WsTalonConfig implements OutputConfig {

    private int m_channel = 0;
    private WsMotorControllers controller;

    /**
     * Construct the Phoenix config.
     * @param channel Controller CAN constant.
     * @param controller Enumeration representing type of controller.
     * @param invert True if motor output should be inverted.
     */
    public WsTalonConfig(int channel, WsMotorControllers controller) {
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
     * Returns true if the motor controller is a Talon.
     * @return True if Talon, false if Victor.
     */
    public WsMotorControllers getType() {
        return controller;
    }

    /**
     * Builds a JSON String describing the Phoenix config.
     * @return Channel number and controller type.
     */
    @Override
    public String toString() {
        StringBuffer buf = new StringBuffer();
        buf.append("{\"channel\": ");
        buf.append(m_channel);
        buf.append(", \"controller\": ");
        buf.append(controller);
        buf.append("}");
        return buf.toString();
    }

}
