package org.wildstang.sample.subsystems.localization;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class LocalizationConstants {
    public static final String kLeftCam = "LeftCam";
    public static final String kRightCam = "RightCam";
    public static final Transform3d kRobotToLeftCam =
                new Transform3d(new Translation3d(-0.282, 0.215, 0.744), new Rotation3d(0, -0.209, -0.175));
    public static final Transform3d kRobotToRightCam =
                new Transform3d(new Translation3d(-0.282, -0.215, 0.744), new Rotation3d(0, -0.209, 0.175));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect correction rate
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1.3, 1.3, Double.MAX_VALUE);  // TODO: tune these values
    public static final Matrix<N3, N1> kMaxStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    
    public static final double BLUE_HUB_X = 4.62;
    public static final double BLUE_HUB_Y = 4.04;

    public static final double RED_HUB_X = 11.92;
    public static final double RED_HUB_Y = 4.04;
}