// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AccelerationLimiterConstants;
import frc.robot.Constants.FieldOrientedDriveConstants;
import frc.robot.Constants.TestingConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldOrientedDrive extends Command {
    private DriveSubsystem driveSubsystem;
    private CommandXboxController xboxController;
    private PIDController bearingPIDController;
    private double goalBearing;
    private double joystickTurnBearing;
    private double joystickMoveMagnitude;
    private double joystickMoveBearing;
    private double robotBearing;
    private double rotSpeed;
    private double xSpeed;
    private double ySpeed;
    private double maximumAcceleration;

    private double previousXSpeed;
    private double previousYSpeed;

    private Boolean slowModeActive = true;
    private Boolean fieldCentricTurning = false;
    // to deal with the onpress toggling nonsense
    private Boolean isAPressed;
    private Boolean isBPressed;

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public FieldOrientedDrive(DriveSubsystem driveSubsystem, CommandXboxController xboxController) {
        this.driveSubsystem = driveSubsystem;
        this.xboxController = xboxController;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        bearingPIDController = new PIDController(FieldOrientedDriveConstants.kFODP, FieldOrientedDriveConstants.kFODI, FieldOrientedDriveConstants.kFODD);
        //setting a tolerance of 2 degrees
        bearingPIDController.setTolerance(Math.PI/360);
        bearingPIDController.setSetpoint(0);
        bearingPIDController.enableContinuousInput(0, 2*Math.PI);
        previousXSpeed=0;
        previousYSpeed=0;
        goalBearing=0;
        slowModeActive=false;

        isAPressed = false;
        isBPressed = false;
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // toggle slow mode on press of A
        if (xboxController.a().getAsBoolean() && !isAPressed){
            slowModeActive = !slowModeActive; // toggle
            isAPressed = true;
        }
        // we ignore anything while its pressed, to avoid rapid switching, by having a "hasToggled" flag while A is pressed
        if (!xboxController.a().getAsBoolean() && isAPressed){ // we only turn isAPressed off if it's on (duh)
            isAPressed = false;
        }

        // ditto for field-centric-turning with B press
        if (xboxController.b().getAsBoolean() && !isBPressed) {
            fieldCentricTurning = !fieldCentricTurning;
            isBPressed = true;
        }
        if (!xboxController.b().getAsBoolean() && isBPressed) {
            isBPressed = false;
        }

        SmartDashboard.putBoolean("Slow Mode Active", slowModeActive);
        SmartDashboard.putBoolean("Field Centric Driving", fieldCentricTurning);

        SmartDashboard.putBoolean("A is on?", isAPressed);
        SmartDashboard.putBoolean("B is on?", isBPressed);

        maximumAcceleration=(slowModeActive ? AccelerationLimiterConstants.maximumAccelerationReduced : AccelerationLimiterConstants.maximumAcceleration);

        //Both joysticks assumes the right to be bearing 0 and then works clockwise from there. To have bearing 0 be in front, the bearing
        //has to be moved back by 90 degrees/ 1/2 PI
        //If right joystick is not being moved retain previous bearing
        if (Math.hypot(xboxController.getRightY(), xboxController.getRightX()) > 0.9) {
            joystickTurnBearing = Math.atan2(xboxController.getRightY(), xboxController.getRightX()) + Math.PI/2;
        }
        //error tolerance of 2 degrees
        if (Math.abs(joystickTurnBearing - goalBearing) > Math.PI/180 * FieldOrientedDriveConstants.bearingTolerance){
            goalBearing = -joystickTurnBearing;
            bearingPIDController.reset();
            bearingPIDController.setSetpoint(goalBearing);
        }
        robotBearing = driveSubsystem.getGyroAngle();
        if (robotBearing < 0){
            //converting to within range 0 to 360 degrees
            robotBearing += 360;
        }
        //converting to radians
        robotBearing = robotBearing / 180 * Math.PI;

        //it looks cooked but that's because the controller is mapped kinda funny
        joystickMoveBearing = Math.atan2(xboxController.getLeftX(), -xboxController.getLeftY());
        joystickMoveBearing = joystickMoveBearing + robotBearing - 2*Math.PI;

        joystickMoveMagnitude = Math.pow(Math.pow(xboxController.getLeftX(), 2) + Math.pow(xboxController.getLeftY(), 2), 0.5);

        // limit acceleration
        xSpeed = joystickMoveMagnitude * Math.cos(joystickMoveBearing) * (slowModeActive ? TestingConstants.maximumSpeedReduced : TestingConstants.maximumSpeed);
        if(xSpeed > previousXSpeed + maximumAcceleration){
            xSpeed = previousXSpeed + maximumAcceleration;
        }
        else if (xSpeed < previousXSpeed - maximumAcceleration){
            xSpeed = previousXSpeed - maximumAcceleration;
        }
        previousXSpeed = xSpeed;

        ySpeed = joystickMoveMagnitude * Math.sin(joystickMoveBearing) * (slowModeActive ? TestingConstants.maximumSpeedReduced : TestingConstants.maximumSpeed);
        if(ySpeed > previousYSpeed + maximumAcceleration){
            ySpeed = previousYSpeed + maximumAcceleration;
        }
        else if (ySpeed < previousYSpeed - maximumAcceleration){
            ySpeed = previousYSpeed - maximumAcceleration;
        }
        previousYSpeed = ySpeed;

        // depending on how you want to control the robot, toggled by button B
        if (fieldCentricTurning) {
            // Datis and Ella prefers this
            rotSpeed = bearingPIDController.calculate(robotBearing);
        } else {
            // William and Sam prefers this
            rotSpeed = -xboxController.getRightX();
        }

        // multiply by slow mode factor
        rotSpeed *= ( 
            slowModeActive ?
            TestingConstants.reducedRotationSpeedRobotOriented :
            TestingConstants.maximumRotationSpeedRobotOriented
        );

        driveSubsystem.drive(xSpeed, -ySpeed, rotSpeed, false);
        
        SmartDashboard.putNumber("Y Speed", ySpeed);
        SmartDashboard.putNumber("X Speed", xSpeed);
        SmartDashboard.putNumber("Rotational Speed", rotSpeed);

        SmartDashboard.putNumber("Robot Heading", robotBearing);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0.0, 0.0, 0.0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}