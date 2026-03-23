package org.wildstang.sample.subsystems.intake;

import org.wildstang.sample.robot.CANConstants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class IntakeConstants {

    public static final int SPIN_LIMIT = 60;
    public static final int SPIN_STALL_LIMIT = 60;

    public static final double DEPLOY_P = 0;
    public static final double DEPLOY_I = 0;
    public static final double DEPLOY_D = 0;
    public static final ClosedLoopSlot DEPLOY_SLOT = ClosedLoopSlot.kSlot0;;
    public static final double DEPLOY_ROTATIONS = 34.3;

    public static final double RETRACT_P = 0;
    public static final double RETRACT_I = 0;
    public static final double RETRACT_D = 0;
    public static final ClosedLoopSlot RETRACT_SLOT = ClosedLoopSlot.kSlot1;
    public static final double RETRACT_ROTATIONS = -1;

    public static final double ROLLER_P = 0.00015;
    public static final double ROLLER_I = 0;
    public static final double ROLLER_D = 0;
    public static final double ROLLER_kS = 0.25;
    public static final double ROLLER_kV = 0.0018;
    public static final double ROLLER_kA = 0;

    public static SparkFlexConfig leftSpinConfig() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(SPIN_STALL_LIMIT, SPIN_LIMIT);
        config.idleMode(IdleMode.kCoast);
        config.inverted(true);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(ROLLER_P, ROLLER_I, ROLLER_D);
        config.closedLoop.feedForward.kS(ROLLER_kS).kV(ROLLER_kV).kA(ROLLER_kA);
        
        return config;
    }

    public static SparkFlexConfig rightSpinConfig() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(SPIN_STALL_LIMIT, SPIN_LIMIT);
        config.follow(CANConstants.INTAKE_SPIN_LEFT, true); // right follows left
        config.idleMode(IdleMode.kCoast);

        return config;
    }
}