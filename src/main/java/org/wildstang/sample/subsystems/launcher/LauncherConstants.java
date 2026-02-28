package org.wildstang.sample.subsystems.launcher;

import org.wildstang.sample.robot.CANConstants;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class LauncherConstants {
    
    public static final double BOTTOM_ANGLE_DEG = 40;
    public static final double TOP_ANGLE_DEG = 76;
    public static final double HOOD_RANGE_DEG = TOP_ANGLE_DEG - BOTTOM_ANGLE_DEG;
    public static final double HOOD_GEAR_RATIO = 60.0 / 1.0;
    public static final double HOOD_CIRCUMFERENCE_RATIO = 350/30;
    public static final double TOTAL_HOOD_RATIO = HOOD_GEAR_RATIO * HOOD_CIRCUMFERENCE_RATIO;
    public static final double HOOD_SCALE = 360;
    public static final double HOOD_P = 0.2;
    public static final double HOOD_I = 0;
    public static final double HOOD_D = 0;
    public static final int HOOD_LIMIT = 20;

    public static final double LAUNCHER_P = 0.0012;
    public static final double LAUNCHER_I = 0;
    public static final double LAUNCHER_D = 0;
    public static final double LAUNCHER_kS = 0.131;
    public static final double LAUNCHER_kV = 0.0019;
    public static final double LAUNCHER_kA = 0.00149;
    public static final int LAUNCHER_LIMIT = 60;
    public static final int LAUNCHER_STALL_LIMIT = 60;

    public static final int PREACCEL_LIMIT = 40;
    public static final int FEED_LIMIT = 40;

    public static SparkMaxConfig hoodConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(HOOD_LIMIT, HOOD_LIMIT);
        config.idleMode(IdleMode.kBrake);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(HOOD_P, HOOD_I, HOOD_D);

        return config;
    }

    public static SparkFlexConfig middleConfig() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(LAUNCHER_STALL_LIMIT, LAUNCHER_LIMIT);
        config.idleMode(IdleMode.kCoast);

        config.encoder.quadratureAverageDepth(4);
        config.encoder.quadratureMeasurementPeriod(1);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(LAUNCHER_P, LAUNCHER_I, LAUNCHER_D);
        config.closedLoop.feedForward.kS(LAUNCHER_kS).kV(LAUNCHER_kV).kA(LAUNCHER_kA);

        return config;
    }

    public static SparkFlexConfig leftConfig() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(LAUNCHER_LIMIT, LAUNCHER_LIMIT);
        config.follow(CANConstants.LAUNCHER_MIDDLE);
        config.idleMode(IdleMode.kCoast);

        return config;
    }

    public static SparkFlexConfig rightConfig() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(LAUNCHER_LIMIT, LAUNCHER_LIMIT);
        config.follow(CANConstants.LAUNCHER_MIDDLE, true);
        config.idleMode(IdleMode.kCoast);

        return config;
    }

    public static SparkMaxConfig preAccelConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(PREACCEL_LIMIT, PREACCEL_LIMIT);
        config.idleMode(IdleMode.kCoast);
        config.inverted(true);

        return config;
    }

    public static SparkMaxConfig feedConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(FEED_LIMIT, FEED_LIMIT);
        config.idleMode(IdleMode.kBrake);
        config.inverted(true);

        return config;
    }
}
