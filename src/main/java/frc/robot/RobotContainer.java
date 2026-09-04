// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.FieldOrientedDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AprilTagPoseEstimator;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.StopRobot;
import frc.robot.commands.MotorsTest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.Optional;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    // Controller port constants
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int MANUAL_LIFT_CONTROLLER_PORT = 1;
    public static final double DRIVE_DEADBAND = 0.1;

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final AprilTagPoseEstimator poseEstimator = new AprilTagPoseEstimator();

    private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    private final XboxController testController = new XboxController(DRIVER_CONTROLLER_PORT);

    private final FieldOrientedDrive fieldOrientedDrive = new FieldOrientedDrive(driveSubsystem, driverController);
    private final MotorsTest motorsTest = new MotorsTest(testController, driveSubsystem);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        if (driverController.x().getAsBoolean()) {
            System.out.println("Gyro reset");
        }
        driverController.x().onTrue( // Reset gyro whenever necessary
            new InstantCommand(() -> driveSubsystem.resetGyro(), driveSubsystem)
        );
    }

    public Command getAutonomousCommand() {
        StopRobot stop = new StopRobot(driveSubsystem);
        return Commands.sequence(
            new PathPlannerAuto("Auto"),
            stop
        );
    }

    public Command getMotorsTestCommand() {
        return motorsTest;
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
