package org.wildstang.sample.subsystems.intake;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConstants {

    public static final int SPIN_LIMIT = 60;
    public static final int SPIN_STALL_LIMIT = 60;


    public static final int DEPLOY_STALL_LIMIT = 30;
    public static final int DEPLOY_LIMIT = 60;
    public static final double DEPLOY_P = 0.1 / 3;
    public static final double DEPLOY_I = 0;
    public static final double DEPLOY_D = 0;
    public static final ClosedLoopSlot DEPLOY_SLOT = ClosedLoopSlot.kSlot0;
    public static final double DEPLOY_RATIO = 5 * 3;
    public static final double DEPLOY_ROTATIONS = 2 * DEPLOY_RATIO;

    public static final double RETRACT_P = 1;
    public static final double RETRACT_I = 0;
    public static final double RETRACT_D = 0;
    public static final ClosedLoopSlot RETRACT_SLOT = ClosedLoopSlot.kSlot1;
    public static final double RETRACT_ROTATIONS = -0.5;

    public static SparkFlexConfig spinConfig() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(SPIN_STALL_LIMIT, SPIN_LIMIT);
        config.idleMode(IdleMode.kBrake);

        return config;
    }

    public static SparkMaxConfig leftConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(DEPLOY_STALL_LIMIT, DEPLOY_LIMIT);
        config.idleMode(IdleMode.kBrake);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(DEPLOY_P, DEPLOY_I, DEPLOY_D, DEPLOY_SLOT);
        config.closedLoop.pid(RETRACT_P, RETRACT_I, RETRACT_D, RETRACT_SLOT);

        return config;
    }
}