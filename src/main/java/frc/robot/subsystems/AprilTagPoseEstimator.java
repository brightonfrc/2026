// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AprilTagPoseEstimator extends SubsystemBase {
  private EstimatedRobotPose prevEstimatedRobotPose = new EstimatedRobotPose(new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)), 0, new ArrayList<PhotonTrackedTarget>(), PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private final PhotonCamera cam;
  private final PhotonPoseEstimator photonPoseEstimator;
  private final VisionSystemSim visionSim;
  private final PhotonCameraSim camSim;
  private boolean streamNamesPrinted = false;

  /** Creates a new AprilTagPoseEstimator. */
  public AprilTagPoseEstimator() {
    // The field from AprilTagFields will be different depending on the game.
    this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    //Forward Camera
    this.cam = new PhotonCamera(Constants.CVConstants.kCameraName);

    // Construct PhotonPoseEstimator
    this.photonPoseEstimator =
        new PhotonPoseEstimator(this.aprilTagFieldLayout, Constants.CVConstants.kRobotToCamera);

    if (RobotBase.isSimulation()) {
      SimCameraProperties cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
      cameraProp.setCalibError(0.25, 0.08);
      cameraProp.setFPS(20);
      cameraProp.setAvgLatencyMs(35);
      cameraProp.setLatencyStdDevMs(5);

      this.visionSim = new VisionSystemSim(Constants.CVConstants.kCameraName);
      this.visionSim.addAprilTags(this.aprilTagFieldLayout);

      this.camSim = new PhotonCameraSim(this.cam, cameraProp);
      this.visionSim.addCamera(this.camSim, Constants.CVConstants.kRobotToCamera);
      // Enable simulated camera streams and wireframe overlay.
      this.camSim.enableRawStream(true);
      this.camSim.enableProcessedStream(true);
      this.camSim.enableDrawWireframe(true);
      this.camSim.setWireframeResolution(0.1);
    } else {
      this.visionSim = null;
      this.camSim = null;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update dashboard entries every loop so sim shows tag info.
    getVisibleTags();
  }

  public void simulationPeriodic(Pose2d robotPose) {
    if (visionSim == null) {
      return;
    }
    if (!streamNamesPrinted) {
      String camName = cam.getName();
      System.out.println("[PhotonVision Sim] Camera streams:");
      System.out.println("  - http://localhost:1181/stream.mjpg?name=" + camName + "-raw");
      System.out.println("  - http://localhost:1181/stream.mjpg?name=" + camName + "-processed");
      streamNamesPrinted = true;
      SmartDashboard.putData("Sim/VisionSystemSimField", visionSim.getDebugField());
    }
    visionSim.getCameraPose(camSim).ifPresent(pose -> {
      SmartDashboard.putString("Sim/CameraPose", pose.toString());
    });
    SmartDashboard.putString("Sim/RobotPose", visionSim.getRobotPose().toString());
    visionSim.update(robotPose);
  }

  /**
   * Get the currently-visible AprilTags on the pitch, as a list.
   */
  public List<PhotonTrackedTarget> getVisibleTags() {
    PhotonPipelineResult latestResult = getLatestResult();
    SmartDashboard.putString("latestResult", latestResult.toString());

    Optional<EstimatedRobotPose> pose =
        photonPoseEstimator.estimateClosestToReferencePose(
            latestResult, this.prevEstimatedRobotPose.estimatedPose);
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
    PhotonPipelineResult latestResult = getLatestResult();
    List<PhotonTrackedTarget> targets = latestResult.targets;
    // SmartDashboard.putNumber("latestResult/count", targets.size());

    for(int i = 0; i < targets.size(); i++) {
      // SmartDashboard.putString("latestResult/"+i, targets.get(i).fiducialId+"@"+targets.get(i).bestCameraToTarget.toString());
    }

    Optional<EstimatedRobotPose> pose =
        photonPoseEstimator.estimateClosestToReferencePose(
            latestResult, this.prevEstimatedRobotPose.estimatedPose);

    if(pose.isPresent()) {
      this.prevEstimatedRobotPose = pose.get();
      return Optional.of(this.prevEstimatedRobotPose);
    }
    return Optional.empty();
  }

  /**
   * Get the robot's estimated global pose (same as getGlobalPose).
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return getGlobalPose();
  }

  /**
   * Get standard deviations for vision pose estimate based on tag count and distance.
   */
  public Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose estimatedPose) {
    int numTags = estimatedPose.targetsUsed.size();
    if (numTags == 0) {
      return VecBuilder.fill(1e9, 1e9, 1e9);
    }

    double totalDist = 0.0;
    for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
      totalDist += target.getBestCameraToTarget().getTranslation().getNorm();
    }
    double avgDist = totalDist / numTags;

    if (numTags == 1 && avgDist > Constants.VisionPoseStdDevConstants.kMaxSingleTagDistanceMeters) {
      return VecBuilder.fill(1e9, 1e9, 1e9);
    }

    double baseXY = numTags == 1
        ? Constants.VisionPoseStdDevConstants.kSingleTagXYStdDev
        : Constants.VisionPoseStdDevConstants.kMultiTagXYStdDev;
    double baseTheta = numTags == 1
        ? Constants.VisionPoseStdDevConstants.kSingleTagThetaStdDev
        : Constants.VisionPoseStdDevConstants.kMultiTagThetaStdDev;

    double scale = 1.0 + (avgDist * avgDist / 25.0);
    return VecBuilder.fill(baseXY * scale, baseXY * scale, baseTheta * scale);
  }

  /**
   * Estimate the robot's field-relative pose using a single AprilTag observation.
   */
  public Optional<Pose3d> estimateFieldToRobotAprilTag(PhotonTrackedTarget target) {
    Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
    if (tagPose.isEmpty()) {
      return Optional.empty();
    }
    Transform3d cameraToRobot = Constants.CVConstants.kRobotToCamera.inverse();
    Pose3d robotPose =
        PhotonUtils.estimateFieldToRobotAprilTag(
            target.getBestCameraToTarget(), tagPose.get(), cameraToRobot);
    return Optional.of(robotPose);
  }

  /**
   * Traditional field-relative pose estimation using yaw/pitch and gyro.
   */
  public Pose2d estimateFieldToRobotTraditional(
      double cameraHeightMeters,
      double targetHeightMeters,
      double cameraPitchRadians,
      double targetPitchRadians,
      Rotation2d targetYaw,
      Rotation2d gyroAngle,
      Pose2d fieldRelativeTargetPose,
      Transform3d cameraToRobot) {
    return PhotonUtils.estimateFieldToRobot(
        cameraHeightMeters,
        targetHeightMeters,
        cameraPitchRadians,
        targetPitchRadians,
        targetYaw,
        gyroAngle,
        fieldRelativeTargetPose,
        new Transform2d(
            cameraToRobot.getTranslation().toTranslation2d(),
            cameraToRobot.getRotation().toRotation2d()));
  }

  /**
   * Traditional field-relative pose estimation using yaw/pitch and gyro (2D transform version).
   */
  public Pose2d estimateFieldToRobotTraditional(
      double cameraHeightMeters,
      double targetHeightMeters,
      double cameraPitchRadians,
      double targetPitchRadians,
      Rotation2d targetYaw,
      Rotation2d gyroAngle,
      Pose2d fieldRelativeTargetPose,
      Transform2d cameraToRobot) {
    return PhotonUtils.estimateFieldToRobot(
        cameraHeightMeters,
        targetHeightMeters,
        cameraPitchRadians,
        targetPitchRadians,
        targetYaw,
        gyroAngle,
        fieldRelativeTargetPose,
        cameraToRobot);
  }

  /**
   * Calculate distance to target based on camera/target heights and pitch angles.
   */
  public double calculateDistanceToTargetMeters(
      double cameraHeightMeters,
      double targetHeightMeters,
      double cameraPitchRadians,
      double targetPitchRadians) {
    return PhotonUtils.calculateDistanceToTargetMeters(
        cameraHeightMeters,
        targetHeightMeters,
        cameraPitchRadians,
        targetPitchRadians);
  }

  /**
   * Estimate a 2D translation from the camera to the target given distance and yaw.
   */
  public Translation2d estimateCameraToTargetTranslation(double distanceMeters, Rotation2d targetYaw) {
    return PhotonUtils.estimateCameraToTargetTranslation(distanceMeters, targetYaw);
  }

  /**
   * Get distance between two poses.
   */
  public double getDistanceToPose(Pose2d robotPose, Pose2d targetPose) {
    return PhotonUtils.getDistanceToPose(robotPose, targetPose);
  }

  /**
   * Get yaw to a pose.
   */
  public Rotation2d getYawToPose(Pose2d robotPose, Pose2d targetPose) {
    return PhotonUtils.getYawToPose(robotPose, targetPose);
  }

  /**
   * Get the transform from the robot to a certain AprilTag; the given AprilTag needs not be seen since
   * right now this function extrapolates from the global pose.
   * @param tagID The integer ID of the AprilTag the robot's position is compared to, from the field.
   * @return The transform from the robot to the tag.
   */
  public Optional<Transform3d> getRobotToTag(int tagID) {

    // Could not work out pose
    Optional<EstimatedRobotPose> globalPose = this.getGlobalPose();
    // SmartDashboard.putString("globalPose", globalPose.toString());
    
    if(globalPose.isEmpty()) return Optional.empty();
    return getRobotToTag(globalPose.get(), tagID);
  }

  private Optional<Transform3d> getRobotToTag(EstimatedRobotPose robotPose, int tagID) {
    // Look for tag
    List<AprilTag> tags = this.aprilTagFieldLayout.getTags();
    for(AprilTag tag : tags) {
      if(tag.ID == tagID) {
        Pose3d poseDifference = tag.pose.relativeTo(robotPose.estimatedPose);
        // double yawRobotToTag = tag.pose.getRotation().getZ() - poseDifferenceFieldCoordinates.getRotation().getZ();
        // Transform3d poseDifferenceTagCoordinates = poseDifferenceFieldCoordinates.;
        return Optional.of(new Transform3d(poseDifference.getTranslation(), poseDifference.getRotation()));
      }
    }
    // SmartDashboard.putString("tags", tagText);

    // Required tag not on field
    return Optional.empty();
  }
/**
 * Get the transform from the robot to the first visible AprilTag.
 * @return The transform from the robot to the tag.
 */
  public Optional<Transform3d> getRobotToSeenTag() {
    Optional<Integer> closestTagId = getClosestVisibleTagId();
    return closestTagId.isPresent() ? getRobotToTag(closestTagId.get()) : Optional.empty();
  }

  /**
   * Get the ID of the closest visible AprilTag from the robot's current estimated pose.
   * Falls back to closest camera-space distance if a global pose cannot be estimated.
   */
  public Optional<Integer> getClosestVisibleTagId() {
    PhotonPipelineResult latestResult = getLatestResult();
    if (!latestResult.hasTargets()) {
      return Optional.empty();
    }

    Optional<EstimatedRobotPose> globalPose = getGlobalPose();
    if (globalPose.isPresent()) {
      Integer closestTagId = null;
      double closestDistance = Double.POSITIVE_INFINITY;
      for (PhotonTrackedTarget target : latestResult.getTargets()) {
        Optional<Transform3d> robotToTag =
            getRobotToTag(globalPose.get(), target.getFiducialId());
        if (robotToTag.isEmpty()) {
          continue;
        }
        double distance = robotToTag.get().getTranslation().getNorm();
        if (closestTagId == null || distance < closestDistance) {
          closestTagId = target.getFiducialId();
          closestDistance = distance;
        }
      }
      if (closestTagId != null) {
        return Optional.of(closestTagId);
      }
    }

    Optional<PhotonTrackedTarget> closestByCamera = selectClosestTargetByCameraDistance(latestResult.getTargets());
    return closestByCamera.map(PhotonTrackedTarget::getFiducialId);
  }

  private PhotonPipelineResult getLatestResult() {
    return cam.getLatestResult();
  }

  /**
   * Expose the latest pipeline result (same timestamp data).
   */
  public PhotonPipelineResult getLatestPipelineResult() {
    return getLatestResult();
  }

  /**
   * Get the best target from the latest pipeline result.
   */
  public Optional<PhotonTrackedTarget> getBestTarget() {
    PhotonPipelineResult result = getLatestResult();
    if (!result.hasTargets()) {
      return Optional.empty();
    }
    return Optional.of(result.getBestTarget());
  }

  /**
   * Get the closest target by distance, breaking ties by lower ambiguity.
   */
  public Optional<PhotonTrackedTarget> getClosestTarget() {
    PhotonPipelineResult result = getLatestResult();
    if (!result.hasTargets()) {
      return Optional.empty();
    }

    Optional<Integer> closestTagId = getClosestVisibleTagId();
    if (closestTagId.isPresent()) {
      for (PhotonTrackedTarget t : result.getTargets()) {
        if (t.getFiducialId() == closestTagId.get()) {
          return Optional.of(t);
        }
      }
    }

    return selectClosestTargetByCameraDistance(result.getTargets());
  }

  /**
   * Get a currently visible target by fiducial ID.
   */
  public Optional<PhotonTrackedTarget> getTargetById(int tagId) {
    PhotonPipelineResult result = getLatestResult();
    if (!result.hasTargets()) {
      return Optional.empty();
    }
    for (PhotonTrackedTarget t : result.getTargets()) {
      if (t.getFiducialId() == tagId) {
        return Optional.of(t);
      }
    }
    return Optional.empty();
  }

  private Optional<PhotonTrackedTarget> selectClosestTargetByCameraDistance(List<PhotonTrackedTarget> targets) {
    PhotonTrackedTarget closest = null;
    double bestDist = Double.POSITIVE_INFINITY;
    double bestAmbiguity = Double.POSITIVE_INFINITY;
    for (PhotonTrackedTarget t : targets) {
      double dist = t.getBestCameraToTarget().getTranslation().getNorm();
      double ambiguity = t.getPoseAmbiguity();
      if (ambiguity < 0) {
        ambiguity = Double.POSITIVE_INFINITY;
      }
      if (closest == null
          || dist < bestDist
          || (dist == bestDist && ambiguity < bestAmbiguity)) {
        closest = t;
        bestDist = dist;
        bestAmbiguity = ambiguity;
      }
    }
    return Optional.ofNullable(closest);
  }

  /**
   * Capture a snapshot from the camera input stream.
   */
  public void takeInputSnapshot() {
    cam.takeInputSnapshot();
  }

  /**
   * Capture a snapshot from the camera output stream.
   */
  public void takeOutputSnapshot() {
    cam.takeOutputSnapshot();
  }

  /**
   * Get the yaw (in degrees) to a specific AprilTag ID from the latest camera results.
   * @param tagId The fiducial ID to look for.
   * @return OptionalDouble containing yaw in degrees if found.
   */
  public OptionalDouble getLatestTargetYawDegrees(int tagId) {
    List<PhotonPipelineResult> results = cam.getAllUnreadResults();
    PhotonPipelineResult result = results.isEmpty()
        ? getLatestResult()
        : results.get(results.size() - 1);

    if (!result.hasTargets()) {
      return OptionalDouble.empty();
    }

    for (PhotonTrackedTarget target : result.getTargets()) {
      if (target.getFiducialId() == tagId) {
        return OptionalDouble.of(target.getYaw());
      }
    }
    return OptionalDouble.empty();
  }

  /**
   * Get the pitch (in degrees) to a specific AprilTag ID from the latest camera results.
   * @param tagId The fiducial ID to look for.
   * @return OptionalDouble containing pitch in degrees if found.
   */
  public OptionalDouble getLatestTargetPitchDegrees(int tagId) {
    List<PhotonPipelineResult> results = cam.getAllUnreadResults();
    PhotonPipelineResult result = results.isEmpty()
        ? getLatestResult()
        : results.get(results.size() - 1);

    if (!result.hasTargets()) {
      return OptionalDouble.empty();
    }

    for (PhotonTrackedTarget target : result.getTargets()) {
      if (target.getFiducialId() == tagId) {
        return OptionalDouble.of(target.getPitch());
      }
    }
    return OptionalDouble.empty();
  }

  private Optional<PhotonTrackedTarget> selectBestTarget(List<PhotonTrackedTarget> targets) {
    if (targets == null || targets.isEmpty()) {
      return Optional.empty();
    }
    PhotonTrackedTarget best = null;
    double bestAmbiguity = Double.POSITIVE_INFINITY;
    double bestDistance = Double.POSITIVE_INFINITY;
    for (PhotonTrackedTarget t : targets) {
      double ambiguity = t.getPoseAmbiguity();
      double distance = t.getBestCameraToTarget().getTranslation().getNorm();
      // Prefer lowest ambiguity; if ambiguity is unavailable (-1), treat as worse.
      if (ambiguity < 0) {
        ambiguity = Double.POSITIVE_INFINITY;
      }
      if (best == null
          || ambiguity < bestAmbiguity
          || (ambiguity == bestAmbiguity && distance < bestDistance)) {
        best = t;
        bestAmbiguity = ambiguity;
        bestDistance = distance;
      }
    }
    return Optional.ofNullable(best);
  }
}
