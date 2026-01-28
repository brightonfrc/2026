// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//

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
import frc.robot.Constants;

public class AprilTagPoseEstimator extends SubsystemBase {
  private EstimatedRobotPose prevEstimatedRobotPose = new EstimatedRobotPose(new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)), 0, new ArrayList<PhotonTrackedTarget>(), PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private final PhotonCamera cam;
  private final PhotonPoseEstimator photonPoseEstimator;

  /** Creates a new AprilTagPoseEstimator. */
  public AprilTagPoseEstimator() {
    // The field from AprilTagFields will be different depending on the game.
    this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    //Forward Camera
    this.cam = new PhotonCamera(Constants.CVConstants.kCameraName);

    // Construct PhotonPoseEstimator
    this.photonPoseEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, Constants.CVConstants.kRobotToCamera);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        // Getting the latest camera result periodically
        if (cam.getLatestResult() != null) {
            SmartDashboard.putString("Camera Status", "Streaming");

            // You can also check if there are visible targets and log them
            getVisibleTags();
        } else {
            SmartDashboard.putString("Camera Status", "Not Streaming");
        }
  }

  /**
   * Get the currently-visible AprilTags on the pitch, as a list.
   */
  public List<PhotonTrackedTarget> getVisibleTags() {
    this.photonPoseEstimator.setReferencePose(this.prevEstimatedRobotPose.estimatedPose);

    SmartDashboard.putString("latestResult", cam.getLatestResult().toString());

    Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(cam.getLatestResult());
    if(pose.isPresent()) {
      this.prevEstimatedRobotPose = pose.get();
      String result = "";
      for(int i = 0; i < this.prevEstimatedRobotPose.targetsUsed.size(); i++) {
        result += ":" + this.prevEstimatedRobotPose.targetsUsed.get(i).fiducialId;
      }
      SmartDashboard.putString("Visible Tags", result);
      return this.prevEstimatedRobotPose.targetsUsed;
    }
    SmartDashboard.putString("Visible Tags", "(none)");
    return new ArrayList<PhotonTrackedTarget>();
  }
  
  /**
   * Get the robot's pose using field-relative coordinates.
   * @return Optional.empty if no AprilTags can be seen and thus the position cannot be derived, or Optional<the robot's pose>.
   */
  public Optional<EstimatedRobotPose> getGlobalPose() {
    this.photonPoseEstimator.setReferencePose(this.prevEstimatedRobotPose.estimatedPose);

    List<PhotonTrackedTarget> targets = cam.getLatestResult().targets;
    // SmartDashboard.putNumber("latestResult/count", targets.size());

    for(int i = 0; i < targets.size(); i++) {
      // SmartDashboard.putString("latestResult/"+i, targets.get(i).fiducialId+"@"+targets.get(i).bestCameraToTarget.toString());
    }

    Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(cam.getLatestResult());

    if(pose.isPresent()) {
      this.prevEstimatedRobotPose = pose.get();
      return Optional.of(this.prevEstimatedRobotPose);
    }
    return Optional.empty();
  }

  /**
   * Get the transform from the robot to a certain AprilTag; the given AprilTag needs not be seen since
   * right now this function extrapolates from the global pose.
   * @param tagID The integer ID of the AprilTag the robot's position is compared to, from the field.
   * @return The transform from the robot to the tag.
   */
  public Optional<Transform3d> getRobotToTag(int tagID) {
  // Get robot pose in field coordinates
  Optional<EstimatedRobotPose> globalPose = getGlobalPose();
  if (globalPose.isEmpty()) return Optional.empty();

  Pose3d robotPose = globalPose.get().estimatedPose;

  // Get tag pose directly by ID
  Optional<Pose3d> tagPoseOpt = aprilTagFieldLayout.getTagPose(tagID);
  if (tagPoseOpt.isEmpty()) return Optional.empty();

  Pose3d tagPose = tagPoseOpt.get();

  // Compute robot -> tag transform
  return Optional.of(new Transform3d(robotPose, tagPose));
}
/**
 * Get the transform from the robot to the first visible AprilTag.
 * @return The transform from the robot to the tag.
 */
  public Optional<Transform3d> getRobotToSeenTag() {
    List<PhotonTrackedTarget> tags = this.getVisibleTags();
    if(tags.size() == 0) {
      return Optional.empty();
    } else {
      PhotonTrackedTarget tag = tags.get(0);
      return this.getRobotToTag(tag.fiducialId);
    }
  }
}
