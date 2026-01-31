// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Field;
import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AprilTagAlignmentConstants;
import frc.robot.Constants.CoralStationAlignConstants;
import frc.robot.Constants.FieldOrientedDriveConstants;
import frc.robot.Constants.TestingConstants;
import frc.robot.subsystems.AprilTagPoseEstimator;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.loggers.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralStationAlign extends Command {
  private DriveSubsystem driveSubsystem;
  private PIDController bearingPIDController;
  private CommandXboxController controller;
  private AprilTagPoseEstimator estimator;
  private PIDController xPIDController;
  private boolean active;

  private GenericLogger logger = new BlankLogger();

  /**
   * Assign logger. refer to RobotContainer.java for the rationale behind this
   * @param logger
   */
  public void assignLogger(GenericLogger logger) {
    this.logger = logger; 
  }

  /** 
   * Creates a new CoralStationAlign, which locks the robot's bearing towards the left coral station
   * Coral station require is decided by the general direction of left joystick
   * cancel by pressing right bumper
   */
  // public CoralStationAlign(DriveSubsystem driveSubsystem, CommandXboxController controller, AprilTagPoseEstimator estimator) {
  //   this.driveSubsystem=driveSubsystem;
  //   this.controller=controller;
  //   this.estimator=estimator;
  //   // Use addRequirements() here to declare subsystem dependencies.
  //   addRequirements(driveSubsystem, estimator);
  // }
  public CoralStationAlign(DriveSubsystem driveSubsystem, CommandXboxController controller) {
    this.driveSubsystem=driveSubsystem;
    this.controller=controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //remember that gyro is flipped
      bearingPIDController=new PIDController(
        CoralStationAlignConstants.kRotP,
        CoralStationAlignConstants.kRotI,
        CoralStationAlignConstants.kRotD
      );
      //left coral station bearing is at 126 to 0, where 0 is forwards for the robot. 
      
      double currentBearing=driveSubsystem.getGyroAngle();
      if (currentBearing<0){
        currentBearing+=360;
      }
      // logger.logDouble("robotBearing", currentBearing);
      if (currentBearing<180){
        // logger.logBool("SetRightCS", false);
        bearingPIDController.setSetpoint(CoralStationAlignConstants.leftCoralStationRot*Math.PI/180);
      }
      else{
        // logger.logBool("SetRightCS", true);
        bearingPIDController.setSetpoint(CoralStationAlignConstants.rightCoralStationRot*Math.PI/180);
      }
      bearingPIDController.setTolerance(FieldOrientedDriveConstants.bearingTolerance);
      bearingPIDController.enableContinuousInput(0, 2*Math.PI);
      // xPIDController=new PIDController(
      //   AprilTagAlignmentConstants.kMoveP, 
      //   AprilTagAlignmentConstants.kMoveI, 
      //   AprilTagAlignmentConstants.kMoveD
      // );
      // xPIDController.setSetpoint(CoralStationAlignConstants.xDisplacement);
      // xPIDController.setSetpoint(CoralStationAlignConstants.xTolerance);
      active=true;
}
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    logger.logDouble("robotBearing", driveSubsystem.getGyroAngle());
    logger.logBool("Coral Station Align Active", true);
    //check to end command
    if (controller.rightBumper().getAsBoolean()){
      //end command the moment rightBumper is pressed
      active=false;
    }
    if (Math.hypot(controller.getRightY(), controller.getRightX())> 0.9) {
      if (controller.getRightX()>0){
        //right stick facing right, which swaps coral station from left to Right. Bearing 240
        bearingPIDController.reset();
        bearingPIDController.setSetpoint(CoralStationAlignConstants.rightCoralStationRot*Math.PI/180);
      }
      else{
        bearingPIDController.reset();
        bearingPIDController.setSetpoint(CoralStationAlignConstants.leftCoralStationRot*Math.PI/180);
      }
    }
    double xSpeed=0;
    double bearing = Math.toRadians(driveSubsystem.getGyroAngle());
    double joystickMoveBearing = Math.atan2(controller.getLeftX(), -controller.getLeftY());
    double joystickMoveMagnitude = Math.hypot(controller.getLeftX(), controller.getLeftY());
    // Optional<Transform3d> pose = estimator.getRobotToSeenTag();
    // if (pose.isEmpty()){
    //   logger.logBool("April Tag in view", false);
    //   xSpeed= joystickMoveMagnitude * Math.cos(joystickMoveBearing) * TestingConstants.maximumSpeedReduced;
    // }
    // else{
    //   logger.logBool("April Tag in view", true);
    //   logger.logDouble("XDisplacement", pose.get().getX());
    //   //reverse xSpeed because drivetrain must drive at positive sped to reduce xdistance
    //   xSpeed= -xPIDController.calculate(pose.get().getX());
    // }
    xSpeed= joystickMoveMagnitude * Math.cos(joystickMoveBearing) * TestingConstants.maximumSpeedReduced;
    double ySpeed= joystickMoveMagnitude * Math.sin(joystickMoveBearing) * TestingConstants.maximumSpeedReduced;
    driveSubsystem.drive(xSpeed, -ySpeed, bearingPIDController.calculate(bearing), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logger.logBool("Coral Station Align Active", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !active;
  }
}
