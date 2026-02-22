package org.wildstang.sample.subsystems.swerve;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ModuleConstants {
    
    /**drive motor gear ratio */
    public static final double DRIVE_RATIO = 5.50;
    /**diameter of drive wheel, in inches */
    public static final double WHEEL_RADIUS = 0.0381;
    /**PID values for drive P */
    public static final double DRIVE_P = 0;
    /**PID values for drive I */
    public static final double DRIVE_I = 0.01;
    /**PID values for drive D */
    public static final double DRIVE_D = 0.1;
    /**PID values for angle P */
    public static final double ANGLE_P = 0.57;
    /**PID values for angle I */
    public static final double ANGLE_I = 0.0;
    /**PID values for angle D */
    public static final double ANGLE_D = 0.0;
    
    /**Drive motor current limit */
    public static final int DRIVE_CURRENT_LIMIT = 50;
    /**Angle motor current limit */
    public static final int ANGLE_CURRENT_LIMIT = 10;

    public static SparkFlexConfig driveConfig() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(DRIVE_CURRENT_LIMIT, DRIVE_CURRENT_LIMIT);
        config.idleMode(IdleMode.kBrake);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(DRIVE_P, DRIVE_I, DRIVE_D);

        return config;
    }

    public static SparkMaxConfig angleConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(ANGLE_CURRENT_LIMIT, ANGLE_CURRENT_LIMIT);
        config.idleMode(IdleMode.kBrake);

        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        config.closedLoop.pid(ANGLE_P, ANGLE_I, ANGLE_D);
        config.closedLoop.positionWrappingEnabled(true);
        config.closedLoop.positionWrappingMaxInput(Math.PI);
        config.closedLoop.positionWrappingMinInput(-Math.PI);

        config.absoluteEncoder.positionConversionFactor(2 * Math.PI);
        config.absoluteEncoder.velocityConversionFactor((2 * Math.PI) / 60);
        config.absoluteEncoder.inverted(true);

        return config;
    }
}