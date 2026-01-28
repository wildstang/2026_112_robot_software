package org.wildstang.sample.robot;

import org.wildstang.framework.core.Subsystems;
import org.wildstang.sample.subsystems.LED.LedController;
import org.wildstang.sample.subsystems.localization.Localization;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;

/**
 * All subsystems are enumerated here.
 * It is used in Robot.java to initialize all subsystems.
 */
public enum WsSubsystems implements Subsystems {

    // enumerate subsystems
    SWERVE_DRIVE("Swerve Drive", SwerveDrive.class),
    LOCALIZATION("Localization", Localization.class),
    LED("Led Controller", LedController.class)
    ;

    /**
     * Do not modify below code, provides template for enumerations.
     * We would like to have a super class for this structure, however,
     * Java does not support enums extending classes.
     */
    
    private String name;
    private Class<?> subsystemClass;
    private boolean enabled;

    /**
     * Initialize name and Subsystem map.
     * @param name Name, must match that in class to prevent errors.
     * @param subsystemClass Class containing Subsystem
     */
    WsSubsystems(String name, Class<?> subsystemClass) {
        this(name, subsystemClass, true);
    }

    /**
     * Initialize name and Subsystem map.
     * @param name Name, must match that in class to prevent errors.
     * @param subsystemClass Class containing Subsystem
     * @param enabled Whether the subsystem should be created by SubsystemManager
     */
    WsSubsystems(String name, Class<?> subsystemClass, boolean enabled) {
        this.name = name;
        this.subsystemClass = subsystemClass;
        this.enabled = enabled;
    }

    /**
     * Returns the name mapped to the subsystem.
     * @return Name mapped to the subsystem.
     */
    @Override
    public String getName() {
        return name;
    }

    /**
     * Returns subsystem's class.
     * @return Subsystem's class.
     */
    @Override
    public Class<?> getSubsystemClass() {
        return subsystemClass;
    }

    /**
     * Returns whether the subsystem is enabled.
     * @return Subsystem's enabled.
     */
    @Override
    public boolean isEnabled() {
        return enabled;
    }
}