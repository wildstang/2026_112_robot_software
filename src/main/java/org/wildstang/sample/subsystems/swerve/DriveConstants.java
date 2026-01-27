package org.wildstang.sample.subsystems.swerve;

import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    // distance from one edge of the robot to the parallel edge
    public static final double ROBOT_LENGTH = Units.inchesToMeters(33);
    public static final double ROBOT_WIDTH = Units.inchesToMeters(22);
    // distance from edge of robot to center of wheel
    public static final double WHEEL_OFFSET = Units.inchesToMeters(1.75);
    // distances from the center of one wheel to another
    public static final double TRACK_LENGTH = ROBOT_LENGTH - WHEEL_OFFSET * 2;
    public static final double TRACK_WIDTH = ROBOT_WIDTH - WHEEL_OFFSET * 2;
    /**speed with which the robot rotates relative to drive speed */
    public static final double ROTATION_SPEED = 0.75;
    /**offset of module 1, the front left module, in radians */
    public static final double FRONT_LEFT_OFFSET = Math.PI / 2.0;
    /**offset of module 2, the front right module, in radians */
    public static final double FRONT_RIGHT_OFFSET = 0;
    /**offset of module 3, the rear left module, in radians */
    public static final double REAR_LEFT_OFFSET = Math.PI;
    /**offset of module 4, the rear right module, in radians */
    public static final double REAR_RIGHT_OFFSET = -Math.PI / 2.0;
    /**deadband of the controller's joysticks */
    public static final double DEADBAND = 0.07;
    /**factor of thrust for the drive trigger */
    public static final double DRIVE_THRUST = 0.4;
    /**factor of derate for the drive trigger */
    public static final double DRIVE_DERATE = 2;
    /**second order correction for rotation plus driving */
    public static final double ROT_CORRECTION_FACTOR = -0.5;
    /**PID values for driveF coefficient of momentum */
    public static final double DRIVE_F_V = 0.19;//free speed of 22.2 ft/s becomes 266.4 in/s
    /**PID values for drive F coefficient of kinetic friction */
    public static final double DRIVE_F_K = 0.20;
    /**Feedforward value for drive rotation */
    public static final double DRIVE_F_ROT = 0.016;
    /**PID values for drive F coefficient of inertia */
    public static final double DRIVE_F_I = 0.0;//
    /**PID values for path tracking position error */
    public static final double POS_P = 2.0;
    /**PID values for path tracking rotation error */
    public static final double ROT_P = 0.55;
    public static final double ROT_D = 2.0;
    public static final String[] POD_NAMES = new String[]{"FL", "FR", "BL", "BR"};
    /**Deadband for deciding if drive is at position target, meters */
    public static final double POS_DB = 0.10;
}
