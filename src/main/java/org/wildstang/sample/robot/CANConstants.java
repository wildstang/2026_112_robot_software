package org.wildstang.sample.robot;

import java.util.Map;


/**
 * CAN Constants are stored here.
 * We primarily use CAN to communicate with Talon motor controllers.
 * These constants must correlate with the IDs set in Phoenix Tuner.
 * Official documentation can be found here:
 * https://phoenix-documentation.readthedocs.io/en/latest/ch08_BringUpCAN.html
 */
public final class CANConstants {

    // Replace these examples.
    // While not independently dangerous if implemented these could have unintended effects.
    //public static final int[] EXAMPLE_PAIRED_CONTROLLERS    = {1,2};
    //public static final int   EXAMPLE_MOTOR_CONTROLLER      = 3;

    //Gyro and CAN sensor values
    public static final int GYRO = 40;

    //swerve constants
    public static final int DRIVE1 = 17; // FL
    public static final int ANGLE1 = 18;
    public static final int DRIVE2 = 15; // FR
    public static final int ANGLE2 = 16;
    public static final int DRIVE3 = 11; // BL
    public static final int ANGLE3 = 12;
    public static final int DRIVE4 = 13; // BR
    public static final int ANGLE4 = 14;

    // intake
    public static final int INTAKE_DEPLOY = 22;
    public static final int INTAKE_SPIN_LEFT = 23;
    public static final int INTAKE_SPIN_RIGHT = 24;

    // shooter
    public static final int FEEDER = 25;
    public static final int PREACCEL = 26;
    public static final int LAUNCHER_LEFT = 27;
    public static final int LAUNCHER_MIDDLE = 28;
    public static final int LAUNCHER_RIGHT = 29;
    public static final int HOOD = 30;

    // only needed for REV logging
    public static final Map<Integer, String> aliasMap = Map.ofEntries(
        Map.entry(DRIVE1, "DRIVE1"),
        Map.entry(ANGLE1, "ANGLE1"),
        Map.entry(DRIVE2, "DRIVE2"),
        Map.entry(ANGLE2, "ANGLE2"),
        Map.entry(DRIVE3, "DRIVE3"),
        Map.entry(ANGLE3, "ANGLE3"),
        Map.entry(DRIVE4, "DRIVE4"),
        Map.entry(ANGLE4, "ANGLE4"),
        
        Map.entry(INTAKE_SPIN_LEFT, "INTAKE_SPIN_LEFT"),
        Map.entry(INTAKE_SPIN_RIGHT, "INTAKE_SPIN_RIGHT"),

        Map.entry(FEEDER, "FEEDER"),
        Map.entry(PREACCEL, "PREACCEL"),
        Map.entry(LAUNCHER_LEFT, "LAUNCHER_LEFT"),
        Map.entry(LAUNCHER_MIDDLE, "LAUNCHER_MIDDLE"),
        Map.entry(LAUNCHER_RIGHT, "LAUNCHER_RIGHT"),
        Map.entry(HOOD, "HOOD")

    );
}