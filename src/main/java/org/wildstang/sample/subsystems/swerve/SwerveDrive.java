package org.wildstang.sample.subsystems.swerve;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.logger.Log;
import org.wildstang.framework.subsystems.swerve.SwerveDriveTemplate;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.sample.robot.CANConstants;
import org.wildstang.sample.robot.WsInputs;
import org.wildstang.sample.robot.WsOutputs;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.localization.Localization;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**Class: SwerveDrive
 * inputs: driver left joystick x/y, right joystick x, right trigger, right bumper, select, face buttons all, gyro
 * outputs: four swerveModule objects
 * description: controls a swerve drive for four swerveModules through autonomous and teleoperated control
 */
public class SwerveDrive extends SwerveDriveTemplate {

    private Localization loc;

    private AnalogInput leftStickX;  // translation joystick x
    private AnalogInput leftStickY;  // translation joystick y
    private AnalogInput rightStickX;  // rot joystick
    private DigitalInput select;  // gyro reset
    private DigitalInput rightStickButton; // Bump mode
    private DigitalInput leftBumper; // Cross mode
    private DigitalInput rightBumper; // Snake mode
    private DigitalInput dpadDown; // TODO: disable vision

    private double xOutput;
    private double yOutput;
    private double rOutput;

    private double xInput;
    private double yInput;
    private double rInput;

    private Pose2d targetPose;
    private ChassisSpeeds targetSpeed;

    public final Pigeon2 gyro = new Pigeon2(CANConstants.GYRO);
    public SwerveModule[] modules;
    private SwerveSignal swerveSignal;
    private WsSwerveHelper swerveHelper = new WsSwerveHelper();

    public SwerveDriveKinematics swerveKinematics;

    public enum DriveState {LAUNCH, TELEOP, AUTO, CROSS, BUMP, SNAKE, FEED};
    public DriveState driveState;
    private Pose2d curPose;
    private boolean shouldCross = false;

    private static final double DEG_TO_RAD = Math.PI / 180.0;
    private static final double RAD_TO_DEG = 180.0 / Math.PI;
    private boolean visionOverride = false;

    public final Field2d m_field = new Field2d();
    private SwerveModuleState[] moduleStates;
    StructArrayPublisher<SwerveModuleState> moduleStatePublisher;
    StructPublisher<ChassisSpeeds> chassisSpeedPublisher, targetSpeedPublisher;
    StructPublisher<Pose2d> targetPosePublisher;

    @Override
    public void init() {
        initInputs();
        initOutputs();
        resetState();
        moduleStates = new SwerveModuleState[] {modules[0].getModuleState(),modules[1].getModuleState(),modules[2].getModuleState(),modules[3].getModuleState()};
        gyro.setYaw(0.0);
        SmartDashboard.putData("Field", m_field);
        moduleStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Swerve Module States", SwerveModuleState.struct).publish();
        chassisSpeedPublisher = NetworkTableInstance.getDefault().getStructTopic("Chassis Speeds", ChassisSpeeds.struct).publish();
        targetSpeedPublisher = NetworkTableInstance.getDefault().getStructTopic("Target Speeds", ChassisSpeeds.struct).publish();
        targetPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Target Pose", Pose2d.struct).publish();
    }

    public void initInputs() {
        leftStickX = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_JOYSTICK_VERTICAL);
        leftStickX.addInputListener(this);
        leftStickY = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_JOYSTICK_HORIZONTAL);
        leftStickY.addInputListener(this);
        rightStickX = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_JOYSTICK_HORIZONTAL);
        rightStickX.addInputListener(this);
        select = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_SELECT);
        select.addInputListener(this);
        rightStickButton = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_JOYSTICK_BUTTON);
        rightStickButton.addInputListener(this);
        leftBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_SHOULDER);
        leftBumper.addInputListener(this);
        rightBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        rightBumper.addInputListener(this);
        dpadDown = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_DOWN);
        dpadDown.addInputListener(this);

    }

    public void initOutputs() {
        //create four swerve modules
        modules = new SwerveModule[]{
            new SwerveModule((WsSpark) Core.getOutputManager().getOutput(WsOutputs.DRIVE1), 
                (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ANGLE1), DriveConstants.FRONT_LEFT_OFFSET),
            new SwerveModule((WsSpark) Core.getOutputManager().getOutput(WsOutputs.DRIVE2), 
                (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ANGLE2), DriveConstants.FRONT_RIGHT_OFFSET),
            new SwerveModule((WsSpark) Core.getOutputManager().getOutput(WsOutputs.DRIVE3), 
                (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ANGLE3), DriveConstants.REAR_LEFT_OFFSET),
            new SwerveModule((WsSpark) Core.getOutputManager().getOutput(WsOutputs.DRIVE4), 
                (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ANGLE4), DriveConstants.REAR_RIGHT_OFFSET)
        };

        double halfWidth = DriveConstants.TRACK_WIDTH / 2;
        double halfLength = DriveConstants.TRACK_LENGTH / 2;
        swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(halfLength, halfWidth),
            new Translation2d(halfLength, -halfWidth),
            new Translation2d(-halfLength, halfWidth),
            new Translation2d(-halfLength, -halfWidth)
        );
        swerveSignal = new SwerveSignal(new double[]{0.0, 0.0, 0.0, 0.0}, new double[]{0.0, 0.0, 0.0, 0.0});
    }

    @Override
    public void initSubsystems(){
        loc = (Localization) Core.getSubsystemManager().getSubsystem(WsSubsystems.LOCALIZATION);
        curPose = loc.getCurrentPose();
    }

    public void resetGyro() {
        if (Core.isBlueAlliance()) {
            loc.setCurrentPose(Pose2d.kZero);
        } else {
            loc.setCurrentPose(new Pose2d(Translation2d.kZero, Rotation2d.k180deg));
        }
    }

    @Override
    public void inputUpdate(Input source) {
        if (source == dpadDown && dpadDown.getValue()) visionOverride = !visionOverride;
        // reset gyro when facing away from alliance station
        if (source == select && select.getValue()) resetGyro();
        
        if (source == leftBumper && leftBumper.getValue()) toggleDriveState(DriveState.CROSS);
        if (source == rightBumper && rightBumper.getValue()) toggleDriveState(DriveState.SNAKE);
        if (source == rightStickButton && rightStickButton.getValue()) toggleDriveState(DriveState.BUMP);

        if (driveState == DriveState.BUMP && Math.abs(rightStickX.getValue()) > 0.1) driveState = DriveState.TELEOP;

        // get x and y speeds
        // joystick axes: +X is to the driver's right, +Y is away from the driver
        // field axes: +X is away from the blue alliance driver station, +Y is to the left from the blue alliance driver station perspective
        xInput = swerveHelper.scaleDeadband(leftStickX.getValue(), DriveConstants.DEADBAND); // joystick y value corresponds to driving forward, i.e. pushing the stick away from the driver (stick +Y) should make the robot drive away from the driver station (field +X)
        yInput = swerveHelper.scaleDeadband(-leftStickY.getValue(), DriveConstants.DEADBAND); // joystick x value corresponds to driving sideways in the opposite direction, i.e. pushing the stick to the drivers left (stick -X) should make the robot drive towards the 

        // reverse x/y directions if on red alliance to match field coordinate system
        if (!Core.isBlueAlliance()) {
            xInput *= -1;
            yInput *= -1;
        }

        xInput *= Math.abs(xInput);
        yInput *= Math.abs(yInput);

        //get rotational joystick
        // joystick positive value corresponds to cw rotation
        // drivetrain positive value corresponds to ccw rotation
        rInput = swerveHelper.scaleDeadband(-rightStickX.getValue(), DriveConstants.DEADBAND); // negate joystick value so positive on the joystick (right) commands a negative rot speed (turn cw) and vice versa

        rInput *= Math.abs(rInput);
    }

    @Override
    public void update() {
        curPose = loc.getCurrentPose();
        double gyroAngle = MathUtil.angleModulus(curPose.getRotation().getRadians()); 
        shouldCross = false;
        double angle;

        switch (driveState) {
            case AUTO:
                setAutoDriveValues(targetSpeed, targetPose);
                break;

            case TELEOP:
                xOutput = xInput;
                yOutput = yInput;
                rOutput = rInput;
                break;

            case LAUNCH:
                xOutput = xInput;
                yOutput = yInput;
                if (!visionOverride) {
                    rOutput = swerveHelper.getRotControl(MathUtil.angleModulus(loc.getTargetAngle()), gyroAngle);
                }
                else {
                    rOutput = rInput;
                }
                break;
            case CROSS:
                shouldCross = true;
                break;
            case SNAKE:
                xOutput = xInput;
                yOutput = yInput;
                rOutput = swerveHelper.getRotControl(MathUtil.angleModulus(Math.atan2(yInput, xInput) - Math.PI) , gyroAngle);
                break;
            case BUMP:
                xOutput = xInput;
                yOutput = yInput;
                if (gyroAngle < 0) {
                    rOutput = swerveHelper.getRotControl(-Math.PI / 2, gyroAngle);
                } else {
                    rOutput = swerveHelper.getRotControl(Math.PI / 2, gyroAngle);
                }
                break;
            case FEED:
                angle = Core.isBlueAlliance() ? Math.PI : 0;
                xOutput = xInput;
                yOutput = yInput;
                rOutput = swerveHelper.getRotControl(angle, gyroAngle);
                break;
        }
        
        rOutput = MathUtil.clamp(rOutput, -1.0, 1.0);
        xOutput = MathUtil.clamp(xOutput, -1.0, 1.0);
        yOutput = MathUtil.clamp(yOutput, -1.0, 1.0);
        this.swerveSignal = shouldCross ? swerveHelper.setCross() : swerveHelper.setDrive(xOutput , yOutput, rOutput, gyroAngle);
        drive();
        putDashboard();
    }

    public void setDriveState(DriveState newState) {
        driveState = newState;
    }

    public void toggleDriveState(DriveState newState) {
        if (newState == driveState) {
            driveState = DriveState.TELEOP;
        } else {
            driveState = newState;
        }
    }

    private void putDashboard() {
        SmartDashboard.putNumber("Gyro Reading", getGyroAngle());
        SmartDashboard.putNumber("X input", xInput);
        SmartDashboard.putNumber("Y input", yInput);
        SmartDashboard.putNumber("X output", xOutput);
        SmartDashboard.putNumber("Y output", yOutput);
        SmartDashboard.putNumber("rOutput", rOutput);
        SmartDashboard.putString("Drive State", driveState.toString());
        SmartDashboard.putBoolean("Blue Alliance", Core.isBlueAlliance());
        SmartDashboard.putBoolean("Pose Estimator Override", visionOverride);
        moduleStates = new SwerveModuleState[] {modules[0].getModuleState(),modules[1].getModuleState(),modules[2].getModuleState(),modules[3].getModuleState()};
        moduleStatePublisher.set(moduleStates);
        chassisSpeedPublisher.set(swerveKinematics.toChassisSpeeds(moduleStates));
        targetSpeedPublisher.set(targetSpeed);
        targetPosePublisher.set(targetPose);
        m_field.setRobotPose(curPose);  // TODO: check if this is needed to post to elastic or if we can use the Localization publisher
    }

    /** sets the drive to teleop/cross, and sets drive motors to coast */
    public void setToTeleop() {
        driveState = DriveState.TELEOP;
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDriveBrake(false);
        }
        rOutput = 0;
        xOutput = 0;
        yOutput = 0;
    }

    /**sets the drive to autonomous */
    public void setToAuto() {
        driveState = DriveState.AUTO;
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDriveBrake(true);
        }
        targetSpeed = new ChassisSpeeds();
        targetPose = curPose;
    }

    /**drives the robot at the current swerveSignal, and displays information for each swerve module */
    private void drive() {
        for (int i = 0; i < modules.length; i++) {
            if (shouldCross) {
                modules[i].runCross(swerveSignal.getSpeed(i), swerveSignal.getAngle(i));
            } else {
                modules[i].run(swerveSignal.getSpeed(i), swerveSignal.getAngle(i));    
            }
            modules[i].displayNumbers(DriveConstants.POD_NAMES[i]);
        }
    }

    @Override
    /**sets autonomous values from the path data file */
    public void setAutoValues(ChassisSpeeds speed, Pose2d pose) {
        targetSpeed = speed;
        targetPose = pose;
    }

    /** Sets output values based on given ChassisSpeeds and target pose, but *does not* 
     * call drive(). This gives the user to option to override certain values if needed.
     * Used for both autonomous mode pathfinding and teleop autoalign.
     */
    private void setAutoDriveValues(ChassisSpeeds speed, Pose2d pose) {
        xOutput = speed.vxMetersPerSecond * DriveConstants.DRIVE_F_K + (pose.getX() - curPose.getX()) * DriveConstants.POS_P;
        yOutput = speed.vyMetersPerSecond * DriveConstants.DRIVE_F_K + (pose.getY() - curPose.getY()) * DriveConstants.POS_P;
        rOutput = speed.omegaRadiansPerSecond * DriveConstants.DRIVE_F_ROT + swerveHelper.getRotControl(MathUtil.angleModulus(pose.getRotation().getRadians()), MathUtil.angleModulus(curPose.getRotation().getRadians()));  // TODO: tune rot controller
    }

    /**
     * Resets the gyro, and sets it the input number of radians
     * Used for starting the match at a non-0 angle
     * @param radians the current value the gyro should read
     */
    public void setGyro(double radians) {
        Log.warn("New gyro value: " + radians);
        StatusCode code = gyro.setYaw(radians * RAD_TO_DEG);
        Log.warn("Gyro Status Code: " + code.getName());
    }

    public double getGyroAngle() {
        return MathUtil.angleModulus(gyro.getYaw().getValueAsDouble() * DEG_TO_RAD);
    }

    public Rotation2d getOdoAngle(){
        return new Rotation2d(getGyroAngle());
    }

    public SwerveModulePosition[] getOdoPosition(){
        return new SwerveModulePosition[]{modules[0].odoPosition(), modules[1].odoPosition(), modules[2].odoPosition(), modules[3].odoPosition()};
    }

    public boolean getVisionOverride() {
        return visionOverride;
    }
    public String currentDriveState() { 
        switch (driveState) {
            case AUTO:
               return "auto";
            case TELEOP:
               return "teleop";
            case LAUNCH:
               return "launch";
            case CROSS:
               return "cross";
            case SNAKE:
               return "snake";
            case BUMP:
               return "bump";
            case FEED:
               return "feed";
        }
                return null;
    }
    
    @Override
    public void resetState() {
        setToTeleop();
    }
    
    @Override
    public void selfTest() {
    }

    @Override
    public String getName() {
        return "Swerve Drive";
    }

    @Override
    public void setAutoHeading(double headingTarget) {
    }

    public void setToLaunch() {
        driveState = DriveState.LAUNCH;
    }
}