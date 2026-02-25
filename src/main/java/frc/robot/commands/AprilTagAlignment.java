package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AprilTagPoseEstimator;
import frc.robot.Constants.AprilTagAlignmentConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation3d;

import java.util.Optional;

public class AprilTagAlignment extends Command {
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
    System.out.println("Start Align");
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
    // System.out.println("Aligning");
    // Using displacement from first visible tag
    Optional<Transform3d> pose = poseEstimator.getRobotToSeenTag();
    SmartDashboard.putNumber("Command/setPoint/X", offsetX);
    SmartDashboard.putNumber("Command/Setpoint/Y", offsetY);

    if (pose.isPresent()) {
      SmartDashboard.putBoolean("Tag in view", true);
      Transform3d transform = pose.get();
      SmartDashboard.putNumber("command/t/x", transform.getTranslation().getX());
      SmartDashboard.putNumber("command/t/y", transform.getTranslation().getY());
      SmartDashboard.putNumber("command/r/yaw", transform.getRotation().getZ());
      Translation3d translation = transform.getTranslation();
      Rotation3d rotation = transform.getRotation();

      double x = translation.getX(); // Forward/Backward
      double y = translation.getY(); // Left/Right

      double yaw = rotation.getZ();  // Rotation (Yaw)

      double rotationOutput = rotationPID.calculate(yaw);

      if(rotationPID.atSetpoint()){ 
        SmartDashboard.putBoolean("Aligned", true);
        // Move towards set point
        // Use PID  to calculate the movement speed needed to reduce error
        double movementSpeed = movementXPID.calculate(x); // 
        double strafeSpeed = movementYPID.calculate(y);   // 
        
        // Drive the robot towards the AprilTag
        SmartDashboard.putNumber("Speed/X", -movementSpeed);
        SmartDashboard.putNumber("Speed/Y", strafeSpeed);
        driveSubsystem.drive(-movementSpeed, strafeSpeed, 0, false); 
        // no need to invert strafespeed because y is also inversed (positive = left) 
        // negative movementSpeed because robot needs to drive forwards (not backwards) in order to reduce xDisplacement
      }
      else{ // If not at set point, rotate towards setpoint
        SmartDashboard.putBoolean("Aligned", false);
        SmartDashboard.putNumber("Speed/rot", rotationOutput);
        driveSubsystem.drive(0, 0, -rotationOutput, false);
      }
    }
    else {
      SmartDashboard.putBoolean("Tag in view", false);
      tagDisappeared = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("End Align");
    driveSubsystem.drive(0, 0, 0, false); // Stop the robot
  }

  @Override
  public boolean isFinished() {
    Boolean atSetpoint = (movementXPID.atSetpoint() && movementYPID.atSetpoint() && rotationPID.atSetpoint());
    System.out.println("At Setpoint " + atSetpoint + "; tagDisappeared " + tagDisappeared);
    return atSetpoint || tagDisappeared; // Stops when within error tolerance
  }
}
