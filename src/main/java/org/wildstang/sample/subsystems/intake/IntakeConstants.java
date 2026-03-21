package org.wildstang.sample.subsystems.intake;

import org.wildstang.sample.robot.CANConstants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class IntakeConstants {

    public static final int SPIN_LIMIT = 60;
    public static final int SPIN_STALL_LIMIT = 60;

    public static final double DEPLOY_P = 0;
    public static final double DEPLOY_I = 0;
    public static final double DEPLOY_D = 0;
    public static final ClosedLoopSlot DEPLOY_SLOT = ClosedLoopSlot.kSlot0;
    public static final double DEPLOY_RATIO = 0;
    public static final double DEPLOY_ROTATIONS = 1 * DEPLOY_RATIO;

    public static final double RETRACT_P = 0;
    public static final double RETRACT_I = 0;
    public static final double RETRACT_D = 0;
    public static final ClosedLoopSlot RETRACT_SLOT = ClosedLoopSlot.kSlot1;
    public static final double RETRACT_ROTATIONS = 0;

    public static SparkFlexConfig leftSpinConfig() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(SPIN_STALL_LIMIT, SPIN_LIMIT);
        config.idleMode(IdleMode.kBrake);
        return config;
    }

    public static SparkFlexConfig rightSpinConfig() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(SPIN_STALL_LIMIT, SPIN_LIMIT);
        config.follow(CANConstants.INTAKE_SPIN_RIGHT, true); // right follows left
        config.idleMode(IdleMode.kBrake);

        return config;
    }
}