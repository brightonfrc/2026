// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagPoseEstimator extends SubsystemBase {
    // TODO: Fill in camera name and position
    // Before using AprilTagPoseEstimator, ensure the camera is calibrated, in 3D mode, and on an AprilTag pipeline at https://photonvision.local:5800
    public static final String CAMERA_NAME = "Arducam_OV9281_USB_Camera"; // Options: [USB]C270_HD_WEBCAM, [Innomaker]Arducam_OV9281_USB_Camera
    // Remember y is sideways, x is forwards for our code
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(new Translation3d(0.330, 0, 0.125), new Rotation3d(0, 0, 0));

    private EstimatedRobotPose prevEstimatedRobotPose = new EstimatedRobotPose(
        new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)),
        0,
        new ArrayList<PhotonTrackedTarget>(),
        PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final PhotonCamera cam;
    private final PhotonPoseEstimator photonPoseEstimator;

    public AprilTagPoseEstimator() {
        // The field from AprilTagFields will be different depending on the game.
        this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Forward Camera
        this.cam = new PhotonCamera(CAMERA_NAME);

        // Construct PhotonPoseEstimator
        this.photonPoseEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, ROBOT_TO_CAMERA);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    // Get the currently-visible AprilTags on the pitch, as a list.
    public List<PhotonTrackedTarget> getVisibleTags() {
        this.photonPoseEstimator.setReferencePose(this.prevEstimatedRobotPose.estimatedPose);

        SmartDashboard.putString("latestResult", cam.getLatestResult().toString());

        Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(cam.getLatestResult());
        if (pose.isPresent()) {
            this.prevEstimatedRobotPose = pose.get();
            String result = "";
            for (int i = 0; i < this.prevEstimatedRobotPose.targetsUsed.size(); i++) {
                result += ":" + this.prevEstimatedRobotPose.targetsUsed.get(i).fiducialId;
            }
            SmartDashboard.putString("Visible Tags", result);
            return this.prevEstimatedRobotPose.targetsUsed;
        }
        SmartDashboard.putString("Visible Tags", "(none)");
        return new ArrayList<PhotonTrackedTarget>();
    }

    // Get the robot's pose using field-relative coordinates.
    // Returns Optional.empty if no AprilTags can be seen, otherwise returns the robot's pose.
    public Optional<EstimatedRobotPose> getGlobalPose() {
        this.photonPoseEstimator.setReferencePose(this.prevEstimatedRobotPose.estimatedPose);

        Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(cam.getLatestResult());

        if (pose.isPresent()) {
            this.prevEstimatedRobotPose = pose.get();
            return Optional.of(this.prevEstimatedRobotPose);
        }
        return Optional.empty();
    }

    // Get the transform from the robot to a certain AprilTag.
    // The given AprilTag need not be seen since this extrapolates from the global pose.
    // tagID: The integer ID of the AprilTag the robot's position is compared to.
    public Optional<Transform3d> getRobotToTag(int tagID) {
        Optional<EstimatedRobotPose> globalPose = this.getGlobalPose();

        if (globalPose.isEmpty()) return Optional.empty();

        // Look for tag
        List<AprilTag> tags = this.aprilTagFieldLayout.getTags();
        for (AprilTag tag : tags) {
            if (tag.ID == tagID) {
                Pose3d poseDifference = tag.pose.relativeTo(globalPose.get().estimatedPose);
                return Optional.of(new Transform3d(poseDifference.getTranslation(), poseDifference.getRotation()));
            }
        }

        // Required tag not on field
        return Optional.empty();
    }

    // Get the transform from the robot to the first visible AprilTag.
    public Optional<Transform3d> getRobotToSeenTag() {
        List<PhotonTrackedTarget> tags = this.getVisibleTags();
        if (tags.size() == 0) {
            return Optional.empty();
        } else {
            PhotonTrackedTarget tag = tags.get(0);
            return this.getRobotToTag(tag.fiducialId);
        }
    }
}
