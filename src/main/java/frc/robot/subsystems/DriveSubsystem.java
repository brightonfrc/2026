// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.loggers.BlankLogger;
import frc.robot.loggers.GenericLogger;
import frc.robot.loggers.loggables.SubsystemBaseWithLogger;
import frc.robot.Constants.AutonPathPlannerConstants;
import frc.robot.Constants.ChoreoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldOrientedDriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.studica.frc.AHRS;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathPlannerPath;


public class DriveSubsystem extends SubsystemBaseWithLogger {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);
  public double getTilt(){
    if(Math.abs(m_gyro.getPitch())>Math.abs(m_gyro.getRoll()))
    {
      return Math.abs(m_gyro.getPitch());
    } else{
      return Math.abs(m_gyro.getRoll());
    }
  }

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()%360),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });
  
  //for Choreo
  // private final PIDController xController = new PIDController(ChoreoConstants.translationkP, ChoreoConstants.translationkI, ChoreoConstants.translationkD);
  // private final PIDController yController = new PIDController(ChoreoConstants.translationkP, ChoreoConstants.translationkI, ChoreoConstants.translationkD);
  // private final PIDController headingController = new PIDController(ChoreoConstants.rotationkP, ChoreoConstants.rotationkI, ChoreoConstants.rotationkD);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      return;
    }
    AutoBuilder.configure(
      this::getPose,
      this::resetOdometry,
      this::getChassisSpeeds,
      (speeds, feedforwards) -> chassisDrive(speeds),
      new PPHolonomicDriveController(
        new PIDConstants(AutonPathPlannerConstants.translationkP, AutonPathPlannerConstants.translationkI, AutonPathPlannerConstants.translationkD),//new PIDConstants(0.4, 0.0, 0.4),
        new PIDConstants(AutonPathPlannerConstants.rotationkP, AutonPathPlannerConstants.rotationkI, AutonPathPlannerConstants.rotationkD)//new PIDConstants(0.4, 0.0, 0.0)
      ),
      config,
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
      );
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  //for Choreo
  // public void followTrajectory(SwerveSample sample) {
  //       // Get the current pose of the robot
  //       Pose2d pose = getPose();

  //       // Generate the next speeds for the robot
  //       ChassisSpeeds speeds = new ChassisSpeeds(
  //           sample.vx + xController.calculate(pose.getX(), sample.x),
  //           sample.vy + yController.calculate(pose.getY(), sample.y),
  //           sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
  //       );

  //       // Apply the generated speeds
  //       drive(speeds.vxMetersPerSecond/DriveConstants.kMaxSpeedMetersPerSecond,speeds.vyMetersPerSecond/DriveConstants.kMaxSpeedMetersPerSecond,0,false);
  //       // drive(speeds.vxMetersPerSecond/DriveConstants.kMaxSpeedMetersPerSecond,speeds.vyMetersPerSecond/DriveConstants.kMaxSpeedMetersPerSecond,speeds.omegaRadiansPerSecond/DriveConstants.kMaxAngularSpeed,false);
  //   }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()%360),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    Pose2d pose= m_odometry.getPoseMeters();
    // double angle =m_gyro.getAngle()%360;
    // if (angle<0){
    //   angle+=360;
    // }
    // pose= new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(angle));
    logger.logDouble("Pose/x", pose.getX());
    logger.logDouble("Pose/y", pose.getY());
    logger.logDouble("Pose/rot", pose.getRotation().getDegrees());
    return pose;
  }
  /**Returns currently-estimated pose without current bearing */
  public Pose2d getPoseChoreo(){
    Pose2d pose=getPose();    
    //for some reason pose is reversed
    return new Pose2d(-pose.getX(), -pose.getY(), Rotation2d.fromDegrees(0));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()%360),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle()%360))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()%360).getDegrees();
  }

  public ChassisSpeeds getChassisSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    );
  }

  public void chassisDrive(ChassisSpeeds speeds){
    double xSpeed = speeds.vxMetersPerSecond;
    double ySpeed = speeds.vyMetersPerSecond;
    double omega = speeds.omegaRadiansPerSecond;
    drive(xSpeed, ySpeed, omega, false);
  }

  
  

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Resets the gyroscope to 0 degrees
   */
  public void resetGyro(){
    m_gyro.reset();
  }

  /**
   * Gets angle of the gyro in degrees
   */
  public double getGyroAngle(){
    return m_gyro.getAngle()%360;
  }
}
