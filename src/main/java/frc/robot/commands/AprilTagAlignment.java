package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AprilTagPoseEstimator;
import frc.robot.Constants.AprilTagAlignmentConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.loggers.*;
import frc.robot.loggers.loggables.CommandWithLogger;

public class AprilTagAlignment extends CommandWithLogger {
  private DriveSubsystem driveSubsystem;
  private AprilTagPoseEstimator poseEstimator;
  private PIDController movementXPID;
  private PIDController movementYPID;
  private double offsetX;
  private double offsetY;
  private Boolean tagDisappeared;

  private PIDController rotationPID;

  public AprilTagAlignment(DriveSubsystem _driveSubsystem, AprilTagPoseEstimator _poseEstimator, double offsetX, double offsetY) {
    driveSubsystem = _driveSubsystem;
    poseEstimator = _poseEstimator;
    addRequirements(_driveSubsystem, _poseEstimator);
    this.offsetX=offsetX;
    this.offsetY=offsetY;
    this.addRequirements(_driveSubsystem, _poseEstimator);
  }

  @Override
  public void initialize() {
    tagDisappeared=false;
    logger.echo("Start Align");
    movementXPID = new PIDController(
      AprilTagAlignmentConstants.kMoveP,
      AprilTagAlignmentConstants.kMoveI,
      AprilTagAlignmentConstants.kMoveD
    );

    movementYPID = new PIDController(
      AprilTagAlignmentConstants.kMoveP,
      AprilTagAlignmentConstants.kMoveI,
      AprilTagAlignmentConstants.kMoveD
    );

    rotationPID = new PIDController(
      AprilTagAlignmentConstants.kTurnP,
      AprilTagAlignmentConstants.kTurnI,
      AprilTagAlignmentConstants.kTurnD
    );
    //at PI the robot is aligned. 
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    rotationPID.setSetpoint(Math.PI);
    movementXPID.setSetpoint(offsetX);
    movementYPID.setSetpoint(offsetY);

    
    // Set tolerances for stopping
    movementXPID.setTolerance(AprilTagAlignmentConstants.errorIntervalPositions);
    movementYPID.setTolerance(AprilTagAlignmentConstants.errorIntervalPositions);
  }
  
  @Override
  public void execute() {
    // logger.echo("Aligning");
    // Using displacement from first visible tag
    Optional<Transform3d> pose = poseEstimator.getRobotToSeenTag();
    logger.log("Command/setPoint/X", offsetX);
    logger.log("Command/Setpoint/Y", offsetY);

    if (pose.isPresent()) {
      logger.log("Tag in view", true);
      Transform3d transform = pose.get();
      logger.log("command/t/x", transform.getTranslation().getX());
      logger.log("command/t/y", transform.getTranslation().getY());
      logger.log("command/r/yaw", transform.getRotation().getZ());
      Translation3d translation = transform.getTranslation();
      Rotation3d rotation = transform.getRotation();

      double x = translation.getX(); // Forward/Backward
      double y = translation.getY(); // Left/Right

      double yaw = rotation.getZ();  // Rotation (Yaw)

      double rotationOutput = rotationPID.calculate(yaw);

      if(rotationPID.atSetpoint()){ 
        logger.log("Aligned", true);
        // Move towards set point
        // Use PID  to calculate the movement speed needed to reduce error
        double movementSpeed = movementXPID.calculate(x); // 
        double strafeSpeed = movementYPID.calculate(y);   // 
        
        // Drive the robot towards the AprilTag
        logger.log("Speed/X", -movementSpeed);
        logger.log("Speed/Y", strafeSpeed);
        driveSubsystem.drive(-movementSpeed, strafeSpeed, 0, false); 
        // no need to invert strafespeed because y is also inversed (positive = left) 
        // negative movementSpeed because robot needs to drive forwards (not backwards) in order to reduce xDisplacement
      }
      else{ // If not at set point, rotate towards setpoint
        logger.log("Aligned", false);
        logger.log("Speed/rot", rotationOutput);
        driveSubsystem.drive(0, 0, -rotationOutput, false);
      }
    }
    else {
      logger.log("Tag in view", false);
      tagDisappeared = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    logger.echo("End Align");
    driveSubsystem.drive(0, 0, 0, false); // Stop the robot
  }

  @Override
  public boolean isFinished() {
    Boolean atSetpoint = (movementXPID.atSetpoint() && movementYPID.atSetpoint() && rotationPID.atSetpoint());
    logger.echo("At Setpoint " + atSetpoint + "; tagDisappeared " + tagDisappeared);
    return atSetpoint || tagDisappeared; // Stops when within error tolerance
  }
}
