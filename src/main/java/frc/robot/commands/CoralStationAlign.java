// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldOrientedDriveConstants;
import frc.robot.Constants.TestingConstants;
import frc.robot.subsystems.DriveSubsystem;

public class CoralStationAlign extends Command {
    // Coral station alignment constants
    public static final double X_DISPLACEMENT = 0.8;
    public static final double X_TOLERANCE = 0.05;
    // remember that back of robot must face coral station
    public static final double LEFT_CORAL_STATION_ROT = 126;
    public static final double RIGHT_CORAL_STATION_ROT = 234;
    public static final double ROT_P = 0.14;
    public static final double ROT_I = 0.01;
    public static final double ROT_D = 0.0;

    private DriveSubsystem driveSubsystem;
    private PIDController bearingPIDController;
    private CommandXboxController controller;
    private boolean active;

    // Creates a new CoralStationAlign, which locks the robot's bearing towards the left coral station.
    // Coral station required is decided by the general direction of left joystick.
    // Cancel by pressing right bumper.
    public CoralStationAlign(DriveSubsystem driveSubsystem, CommandXboxController controller) {
        this.driveSubsystem = driveSubsystem;
        this.controller = controller;
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // remember that gyro is flipped
        bearingPIDController = new PIDController(ROT_P, ROT_I, ROT_D);
        // left coral station bearing is at 126 to 0, where 0 is forwards for the robot.

        double currentBearing = driveSubsystem.getGyroAngle();
        if (currentBearing < 0) {
            currentBearing += 360;
        }
        if (currentBearing < 180) {
            bearingPIDController.setSetpoint(LEFT_CORAL_STATION_ROT * Math.PI / 180);
        } else {
            bearingPIDController.setSetpoint(RIGHT_CORAL_STATION_ROT * Math.PI / 180);
        }
        bearingPIDController.setTolerance(FieldOrientedDriveConstants.BEARING_TOLERANCE);
        bearingPIDController.enableContinuousInput(0, 2 * Math.PI);
        active = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("robotBearing", driveSubsystem.getGyroAngle());
        SmartDashboard.putBoolean("Coral Station Align Active", true);
        // check to end command
        if (controller.rightBumper().getAsBoolean()) {
            // end command the moment rightBumper is pressed
            active = false;
        }
        if (Math.hypot(controller.getRightY(), controller.getRightX()) > 0.9) {
            if (controller.getRightX() > 0) {
                // right stick facing right, which swaps coral station from left to Right. Bearing 240
                bearingPIDController.reset();
                bearingPIDController.setSetpoint(RIGHT_CORAL_STATION_ROT * Math.PI / 180);
            } else {
                bearingPIDController.reset();
                bearingPIDController.setSetpoint(LEFT_CORAL_STATION_ROT * Math.PI / 180);
            }
        }
        double bearing = Math.toRadians(driveSubsystem.getGyroAngle());
        double joystickMoveBearing = Math.atan2(controller.getLeftX(), -controller.getLeftY());
        double joystickMoveMagnitude = Math.hypot(controller.getLeftX(), controller.getLeftY());
        double xSpeed = joystickMoveMagnitude * Math.cos(joystickMoveBearing) * TestingConstants.MAXIMUM_SPEED_REDUCED;
        double ySpeed = joystickMoveMagnitude * Math.sin(joystickMoveBearing) * TestingConstants.MAXIMUM_SPEED_REDUCED;
        driveSubsystem.drive(xSpeed, -ySpeed, bearingPIDController.calculate(bearing), false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Coral Station Align Active", false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !active;
    }
}
