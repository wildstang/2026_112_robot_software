package org.wildstang.sample.subsystems.swerve;

import org.wildstang.hardware.roborio.outputs.WsSpark;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

    private double target;
    private double drivePower;
    private double chassisOffset;

    private WsSpark driveMotor;
    private WsSpark angleMotor;
    private SparkAbsoluteEncoder absEncoder;

    /** Class: SwerveModule
     *  controls a single swerve pod, featuring two motors and one offboard sensor
     * @param driveMotor canSparkMax of the drive motor
     * @param angleMotor canSparkMax of the angle motor
     * @param canCoder canCoder offboard encoder
     * @param offset double value of cancoder when module is facing forward
     */
    public SwerveModule(WsSpark driveMotor, WsSpark angleMotor, double offset) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.absEncoder = angleMotor.getController().getAbsoluteEncoder();

        chassisOffset = offset;
    }

    /** return double for cancoder position 
     * @return double for cancoder value (radians)
    */
    public double getAngle() {
        return MathUtil.angleModulus(absEncoder.getPosition() + chassisOffset);
    }

    /** displays module information, needs the module name from super 
     * @param name the name of this module
    */
    public void displayNumbers(String name) {
        SmartDashboard.putNumber(name + " true angle", getAngle());
        SmartDashboard.putNumber(name + " true target", target);
        SmartDashboard.putNumber(name + " raw angle", absEncoder.getPosition());
        SmartDashboard.putNumber(name + " raw target", MathUtil.angleModulus(target - chassisOffset));
        SmartDashboard.putNumber(name + " NEO drive power", drivePower);
        SmartDashboard.putNumber(name + " NEO drive position", driveMotor.getPosition());
        SmartDashboard.putBoolean(name + " Drive Direction", getDirection(target));
    }

    /** resets drive encoder */
    public void resetDriveEncoders() {
        driveMotor.resetEncoder();
    }

    /**sets drive to brake mode if true, coast if false 
     * @param isBrake true for brake, false for coast
    */
    public void setDriveBrake(boolean isBrake) {
        SparkFlexConfig config = ModuleConstants.driveConfig();
        if(!isBrake) {
            config.idleMode(IdleMode.kCoast);
        }
        driveMotor.configure(config);
    }

    /** runs module at double power [0,1] and robot centric radian angle 
     * @param power power [0, 1] to run the module at
     * @param angle angle to run the robot at, radians
    */
    public void run(double power, double angle) {
        this.drivePower = power;
        this.target = angle;
        if (Math.abs(power) < 0.01) {
            runAtPower(power);
        }
        else if (getDirection(angle)) {
            runAtPower(power);
            runAtAngle(angle);
        }
        else {
            runAtPower(-power);
            runAtAngle(angle + Math.PI);
        }
    }

    public void runCross(double power, double angle) {
        this.drivePower = power;
        this.target = angle;
        if (getDirection(angle)) {
            runAtPower(power);
            runAtAngle(angle);
        }
        else {
            runAtPower(-power);
            runAtAngle(angle + Math.PI);
        }
    }

    /**runs at specified robot centric angle 
     * @param angle angle to run the module at
    */
    private void runAtAngle(double angle) {
        angleMotor.setPosition(MathUtil.angleModulus(angle - chassisOffset));
    }

    /**runs module drive at specified power [-1, 1] 
     * @param power the power to run the module at, [-1, 1]
    */
    public void runAtPower(double power) {
        driveMotor.setSpeed(power);
    }

    /** returns drive encoder distance in meters 
     * @return double drive encoder distance in meters
    */
    public double getPosition() {
        return driveMotor.getPosition() * ModuleConstants.WHEEL_RADIUS * 2.0 * Math.PI / ModuleConstants.DRIVE_RATIO;
    }

    /**returns raw drive encoder value, rotations
     * @return drive encoder value, rotations
     */
    public double getRawEncoderValue() {
        return driveMotor.getPosition();
    }

    /**determines if it is faster to travel towards angle at positive power (true), or away from angle with negative power (false) 
     * @param angle the angle you are moving towards
     * @return boolean whether you should move towards that angle or the opposite
    */
    public boolean getDirection(double angle) {
        double angleErr = Math.abs(MathUtil.angleModulus(angle - getAngle()));
        return Math.abs(angleErr) <= (Math.PI / 2.0);
    }

    public WsSpark getDriveMotor() {
        return driveMotor;
    }

    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(driveMotor.getVelocity() * 2.0 * Math.PI * (ModuleConstants.WHEEL_RADIUS) / 60.0, Rotation2d.fromRadians(getAngle()));
    }
    public SwerveModulePosition odoPosition(){
        return new SwerveModulePosition(getPosition(), Rotation2d.fromRadians(getAngle()));
    }
}