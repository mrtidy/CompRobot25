package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * Represents a physical Limelight camera exposing the getPoseEstimate based on
 * given heading and AprilTag in view.
 */
public class LimelightDevice {

    // name of the Limelight used in NetworkTable
    private final String name;

    // AprilTag ID visible; -1 if not currently visible
    private final NetworkTableEntry tid;

    public LimelightDevice(String limelightName) {
        name = limelightName;

        var networkTable = NetworkTableInstance.getDefault().getTable(name);
        tid = networkTable.getEntry("tid");
    }

    public String getName() {
        return name;
    }

    public long getAprilTagId() {
        return tid.getInteger(-1L);
    }

    /**
     * Gets the pose estimate of the robot based on a reading from the current
     * Limelight
     * @param headingDegress The degree of the robot's current heading
     * @return An estimate based on any tag readings from the Limelight
     */
    public PoseEstimate getPoseEstimate(double headingDegress) {
        updateDashboard(headingDegress);

        // LimelightHelpers comes from https://github.com/LimelightVision/limelightlib-wpijava
        // only the yaw from the gyro is necessary here so rest of params are 0;
        // docs say SetRobotOrientation must be called before getBotPostEsimate
        LimelightHelpers.SetRobotOrientation(getName(), headingDegress, 0.0, 0.0, 0.0, 0.0, 0.0);

        // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
        // > For 2024 and beyond, the origin of your coordinate system should
        // > always be the "blue" origin. FRC teams should always use
        // > botpose_orb_wpiblue for pose-related functionality
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }

    private void updateDashboard(double headingDegrees) {
        SmartDashboard.putNumber(getName() + "-Limelight-AprilTagId", getAprilTagId());
        SmartDashboard.putNumber(getName() + "-Limelight-HeadingDegrees", headingDegrees);
    }
}