// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.CoralStationAlign;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AprilTagPoseEstimator;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.StopRobot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    // Controller port constants
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int MANUAL_LIFT_CONTROLLER_PORT = 1;
    public static final double DRIVE_DEADBAND = 0.1;

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final AprilTagPoseEstimator poseEstimator = new AprilTagPoseEstimator();

    private boolean active = false;

    private final CommandXboxController driverController =
        new CommandXboxController(DRIVER_CONTROLLER_PORT);
    private final CommandPS4Controller manualLiftController = new CommandPS4Controller(MANUAL_LIFT_CONTROLLER_PORT);
    private final FieldOrientedDrive fieldOrientedDrive = new FieldOrientedDrive(driveSubsystem, driverController);

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(fieldOrientedDrive);
        configureBindings();
    }

    public static boolean fieldRelative = true;

    private void configureBindings() {
        if (driverController.x().getAsBoolean()) {
            System.out.println("Gyro reset");
        }
        driverController.x().onTrue( // Reset gyro whenever necessary
            new InstantCommand(() -> driveSubsystem.resetGyro(), driveSubsystem)
        );
        driverController.leftBumper().onTrue(new CoralStationAlign(driveSubsystem, driverController));
    }

    public Command getAutonomousCommand() {
        StopRobot stop = new StopRobot(driveSubsystem);
        return Commands.sequence(
            new PathPlannerAuto("Auto"),
            stop
        );
    }

    public void setUpDefaultCommand() {
        driveSubsystem.setDefaultCommand(fieldOrientedDrive);
    }

    public void resetBearings() {
        driveSubsystem.resetOdometry(driveSubsystem.getPose());
        driveSubsystem.resetGyro();
    }

    public void resetGyro() {
        driveSubsystem.resetGyro();
    }

    public Pose2d getPose() {
        return driveSubsystem.getPose();
    }

    // TODO: Delete
    public void printPose() {
        Optional<Transform3d> opt = poseEstimator.getRobotToSeenTag();
        if (opt.isPresent()) {
            Transform3d r2t = opt.get();
            SmartDashboard.putNumber("robot2tag/t/x", r2t.getTranslation().getX());
            SmartDashboard.putNumber("robot2tag/t/y", r2t.getTranslation().getY());
            SmartDashboard.putNumber("robot2tag/r/yaw", r2t.getRotation().getZ());
        }
    }
}
