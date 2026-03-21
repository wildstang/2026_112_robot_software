package org.wildstang.sample.subsystems.localization;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;

import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.StructArrayPublisher;

public class Localization implements Subsystem {
    
    private SwerveDrive drive;
    private SwerveDrivePoseEstimator estimator;
    private PhotonPoseEstimator leftEstimator, rightEstimator;
    private PhotonCamera leftCam, rightCam;
    private Matrix<N3, N1> curStdDevs;
    private Pose2d currentPose;
    StructPublisher<Pose2d> posePublisher, bestPosePublisher, leftCamPublisher, rightCamPublisher;
    StructArrayPublisher<Pose2d> leftVisTargetPublisher, rightVisTargetPublisher;
    StructArrayPublisher<SwerveModulePosition> modulePosPublisher;
    StructPublisher<Rotation2d> odoAngPublisher;
    Optional<EstimatedRobotPose> visionEst;
    List<PhotonTrackedTarget> targets;
    Pose2d[] visTargets;
    Double[] targetAmbiguity;
    public boolean leftHasEst, rightHasEst = false;

    @Override
    public void init() {
        modulePosPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("odo position", SwerveModulePosition.struct).publish();
        odoAngPublisher = NetworkTableInstance.getDefault().getStructTopic("odo angle", Rotation2d.struct).publish();
        currentPose = new Pose2d();
        posePublisher = NetworkTableInstance.getDefault().getStructTopic("Pose Estimator", Pose2d.struct).publish();
        bestPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Best Pose Target", Pose2d.struct).publish();

        leftCam = new PhotonCamera(LocalizationConstants.kLeftCam);
        leftEstimator = new PhotonPoseEstimator(LocalizationConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, LocalizationConstants.kRobotToLeftCam);
        leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        leftCamPublisher = NetworkTableInstance.getDefault().getStructTopic("Left Cam Pose Estimator", Pose2d.struct).publish();
        leftVisTargetPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Left Vision Targets", Pose2d.struct).publish();

        rightCam = new PhotonCamera(LocalizationConstants.kRightCam);
        rightEstimator = new PhotonPoseEstimator(LocalizationConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, LocalizationConstants.kRobotToRightCam);
        rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        rightCamPublisher = NetworkTableInstance.getDefault().getStructTopic("Right Cam Pose Estimator", Pose2d.struct).publish();
        rightVisTargetPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Right Vision Targets", Pose2d.struct).publish();
    }

    @Override
    public void initSubsystems() {
        drive = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        estimator = new SwerveDrivePoseEstimator(drive.swerveKinematics, drive.getOdoAngle(), drive.getOdoPosition(), currentPose);
    }

    @Override
    public void inputUpdate(Input source) {
    }

    @Override
    public void update() {
        // Update pose estimator with drivetrain odometry values
        estimator.update(drive.getOdoAngle(), drive.getOdoPosition());
        modulePosPublisher.set(drive.getOdoPosition());
        odoAngPublisher.set(drive.getOdoAngle());

        // Update pose estimator with vision estimates
        leftHasEst |= processPVResults(leftCam, leftEstimator, leftVisTargetPublisher, leftCamPublisher);
        rightHasEst |= processPVResults(rightCam, rightEstimator, rightVisTargetPublisher, rightCamPublisher);

        // Get current pose estimate after all updates
        currentPose = estimator.getEstimatedPosition();
        putDashboard();
    }

    private boolean processPVResults(PhotonCamera cam, PhotonPoseEstimator camEstimator, StructArrayPublisher<Pose2d> visTargetPublisher, StructPublisher<Pose2d> camPosePublisher){
        // Update pose estimator with camera
        for (var change : cam.getAllUnreadResults()) {
            // create List of targets to publish
            targets = change.getTargets();
            visTargets = new Pose2d[targets.size()];
            targetAmbiguity = new Double[targets.size()];
            for (int i = 0; i < targets.size(); i++){
                visTargets[i] = LocalizationConstants.kTagLayout.getTagPose(targets.get(i).getFiducialId()).get().toPose2d();
                targetAmbiguity[i] = targets.get(i).getPoseAmbiguity();
            }
            visTargetPublisher.set(visTargets, (long) (1_000_000 * change.getTimestampSeconds()));

            // update pose estimator
            visionEst = camEstimator.update(change);
            if (visionEst.isPresent()) {
                camPosePublisher.set(visionEst.get().estimatedPose.toPose2d(), (long) (1_000_000 * visionEst.get().timestampSeconds));
                updateEstimationStdDevs(visionEst, targets);
                estimator.addVisionMeasurement(visionEst.get().estimatedPose.toPose2d(), visionEst.get().timestampSeconds, curStdDevs);
            } else {
                camPosePublisher.set(null, (long) (1_000_000 * change.getTimestampSeconds()));
            }
        }
        return visionEst != null ? visionEst.isPresent() : false;
    }

    private void putDashboard () {
        posePublisher.set(currentPose);
        SmartDashboard.putBoolean("Left Has Est", leftHasEst);
        SmartDashboard.putBoolean("Right Has Est", rightHasEst);
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = LocalizationConstants.kSingleTagStdDevs;
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = LocalizationConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                int tID = tgt.getFiducialId();
                // filter out any poses using barge tags
                if (tID == 4 || tID == 5 || tID == 14 || tID == 15) {
                    curStdDevs = LocalizationConstants.kMaxStdDevs;
                    return;
                }
                // reject any poses with high ambiguity
                if (tgt.getPoseAmbiguity() > 0.2) {
                    curStdDevs = LocalizationConstants.kMaxStdDevs;
                    return;
                }
                var tagPose = LocalizationConstants.kTagLayout.getTagPose(tID);
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = LocalizationConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                estStdDevs = estStdDevs.times(1.0 / numTags);
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4) estStdDevs = LocalizationConstants.kMaxStdDevs;
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));  // TODO: tune std devs
                curStdDevs = estStdDevs;
            }
        }
    }

    public Pose2d getCurrentPose(){
        return currentPose;
    }

    public void setCurrentPose(Pose2d newPose){
        estimator.resetPosition(drive.getOdoAngle(), drive.getOdoPosition(), newPose);
        leftEstimator.setLastPose(newPose);
        rightEstimator.setLastPose(newPose);
        leftHasEst = false;
        rightHasEst = false;
    }

    private Translation2d getAllianceHub() {
        double xTarget; 
        double yTarget;

        if (Core.isBlueAlliance()) {
            xTarget = LocalizationConstants.BLUE_HUB_X;
            yTarget = LocalizationConstants.BLUE_HUB_Y;
        } else {
            xTarget = LocalizationConstants.RED_HUB_X;
            yTarget = LocalizationConstants.RED_HUB_Y;
        }

        double deltaX = xTarget - currentPose.getX();
        double deltaY = yTarget - currentPose.getY();
        
        Translation2d translation = new Translation2d(deltaX, deltaY);
        return translation;
    }

    /**
     * Calculates the distance to the alliance's hub.
     * @return Distance to the hub in meters.
     */
    public double getAllianceHubDistance() {
        return getAllianceHub().getNorm();
    }

    /**
     * Calculates the angle to the alliance's hub.
     * @return Angle to the hub in radians.
     */
    public double getAllianceHubAngle() {
        if (!Core.isBlueAlliance()) {
            return getAllianceHub().getAngle().getRadians() + Math.PI;
        } else {
            return getAllianceHub().getAngle().getRadians();
        }

        //return currentPose.getRotation().plus(robotCentric).getRadians();
    }

    private Translation2d getTarget() {
        double xTarget = 0;
        if (Core.isBlueAlliance() && currentPose.getX() > LocalizationConstants.BLUE_HUB_X) {
            xTarget = LocalizationConstants.BLUE_HUB_X;
        }
        else if (Core.isRedAlliance() && currentPose.getX() < LocalizationConstants.RED_HUB_X) {
            xTarget = LocalizationConstants.RED_HUB_X;
        }
        else {
            return getAllianceHub();
        }

        double yTarget = LocalizationConstants.BLUE_HUB_Y;
        if (currentPose.getY() < LocalizationConstants.BLUE_HUB_Y) {
            yTarget /= 2;
        }
        else {
            yTarget *= 1.5;
        }

        double deltaX = xTarget - currentPose.getX();
        double deltaY = yTarget - currentPose.getY();
        
        Translation2d translation = new Translation2d(deltaX, deltaY);
        return translation;
    }

    /**
     * Calculates the distance to the target.
     * The target is the alliance's hub if in the alliance's zone, otherwise it is halfway between the hub and the wall.
     * @return Distance to the target in meters.
     */
    public double getTargetDistance() {
        return getTarget().getNorm();
    }

    /**
     * Calculates the angle to the target.
     * The target is the alliance's hub if in the alliance's zone, otherwise it is halfway between the hub and the wall.
     * @return Angle to the target in radians.
     */
    public double getTargetAngle() {
        Rotation2d robotCentric = getTarget().getAngle();
        return currentPose.getRotation().plus(robotCentric).getRadians();
    }

    @Override
    public void resetState() {
    }

    @Override
    public void selfTest() {
    }

    @Override
    public String getName() {
        return "Localization";
    }
    
}
