package org.wildstang.framework.subsystems.swerve;

import org.wildstang.framework.subsystems.Subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class SwerveDriveTemplate implements Subsystem{

    public abstract void setAutoHeading(double headingTarget);

    public abstract void setGyro(double degrees);

    public abstract void setToAuto();

    public abstract void setToTeleop();

    public abstract void setAutoValues(ChassisSpeeds speed, Pose2d pose);
}
