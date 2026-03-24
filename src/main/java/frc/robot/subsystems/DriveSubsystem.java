// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;


public class DriveSubsystem extends SubsystemBase {
    // PathPlanner autonomous PID constants
    public static final double AUTON_TRANSLATION_KP = 0.4;
    public static final double AUTON_TRANSLATION_KI = 0;
    public static final double AUTON_TRANSLATION_KD = 0.405;
    public static final double AUTON_ROTATION_KP = 0.55;
    public static final double AUTON_ROTATION_KI = 0.0;
    public static final double AUTON_ROTATION_KD = 0.2;

    // Create MAXSwerveModules
    private final MAXSwerveModule frontLeft = new MAXSwerveModule(
        DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
        DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
        DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule frontRight = new MAXSwerveModule(
        DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
        DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
        DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule rearLeft = new MAXSwerveModule(
        DriveConstants.REAR_LEFT_DRIVING_CAN_ID,
        DriveConstants.REAR_LEFT_TURNING_CAN_ID,
        DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule rearRight = new MAXSwerveModule(
        DriveConstants.REAR_RIGHT_DRIVING_CAN_ID,
        DriveConstants.REAR_RIGHT_TURNING_CAN_ID,
        DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

    // The gyro sensor
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);

    public double getTilt() {
        if (Math.abs(gyro.getPitch()) > Math.abs(gyro.getRoll())) {
            return Math.abs(gyro.getPitch());
        } else {
            return Math.abs(gyro.getRoll());
        }
    }

    // Odometry class for tracking robot pose
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        DriveConstants.DRIVE_KINEMATICS,
        Rotation2d.fromDegrees(gyro.getAngle() % 360),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });

    public DriveSubsystem() {
        RobotConfig config;
        try {
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
                new PIDConstants(AUTON_TRANSLATION_KP, AUTON_TRANSLATION_KI, AUTON_TRANSLATION_KD),
                new PIDConstants(AUTON_ROTATION_KP, AUTON_ROTATION_KI, AUTON_ROTATION_KD)
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

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(
            Rotation2d.fromDegrees(gyro.getAngle() % 360),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            });
    }

    // Returns the currently-estimated pose of the robot.
    public Pose2d getPose() {
        Pose2d pose = odometry.getPoseMeters();
        SmartDashboard.putNumber("Pose/x", pose.getX());
        SmartDashboard.putNumber("Pose/y", pose.getY());
        SmartDashboard.putNumber("Pose/rot", pose.getRotation().getDegrees());
        return pose;
    }

    // Returns currently-estimated pose without current bearing.
    public Pose2d getPoseChoreo() {
        Pose2d pose = getPose();
        // for some reason pose is reversed
        return new Pose2d(-pose.getX(), -pose.getY(), Rotation2d.fromDegrees(0));
    }

    // Resets the odometry to the specified pose.
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(gyro.getAngle() % 360),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            },
            pose);
    }

    // Drives the robot using joystick info.
    // xSpeed: Speed in x direction (forward). ySpeed: Speed in y direction (sideways).
    // rot: Angular rate. fieldRelative: Whether speeds are relative to the field.
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
        double ySpeedDelivered = ySpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
        double rotDelivered = rot * DriveConstants.MAX_ANGULAR_SPEED;

        var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                    Rotation2d.fromDegrees(gyro.getAngle() % 360))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        rearLeft.setDesiredState(swerveModuleStates[2]);
        rearRight.setDesiredState(swerveModuleStates[3]);
    }

    // Sets the wheels into an X formation to prevent movement.
    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    // Sets the swerve ModuleStates.
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);
    }

    // Resets the drive encoders to currently read a position of 0.
    public void resetEncoders() {
        frontLeft.resetEncoders();
        rearLeft.resetEncoders();
        frontRight.resetEncoders();
        rearRight.resetEncoders();
    }

    // Zeroes the heading of the robot.
    public void zeroHeading() {
        gyro.reset();
    }

    // Returns the heading of the robot in degrees, from -180 to 180.
    public double getHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle() % 360).getDegrees();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            rearLeft.getState(),
            rearRight.getState()
        );
    }

    public void chassisDrive(ChassisSpeeds speeds) {
        double xSpeed = speeds.vxMetersPerSecond;
        double ySpeed = speeds.vyMetersPerSecond;
        double omega = speeds.omegaRadiansPerSecond;
        drive(xSpeed, ySpeed, omega, false);
    }

    // Returns the turn rate of the robot in degrees per second.
    public double getTurnRate() {
        return gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
    }

    // Resets the gyroscope to 0 degrees.
    public void resetGyro() {
        gyro.reset();
    }

    // Gets angle of the gyro in degrees.
    public double getGyroAngle() {
        return gyro.getAngle() % 360;
    }
}
