// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AccelerationLimiterConstants;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.FieldOrientedDriveConstants;
import frc.robot.Constants.TestingConstants;
import frc.robot.Constants.ControlConstants.RightJoystickModes;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/** An example command that uses an example subsystem. */
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
    private Boolean hasToggled;

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
        hasToggled=false;
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (xboxController.a().getAsBoolean()&&hasToggled==false){
            slowModeActive=!slowModeActive;
            hasToggled=true;
        }
        if(xboxController.a().getAsBoolean()==false){
            //button no longer pressed, able to toggle slow mode now
            hasToggled=false;
        }
        SmartDashboard.putBoolean("Slow Mode Active", slowModeActive);
        maximumAcceleration=(slowModeActive ? AccelerationLimiterConstants.maximumAccelerationReduced : AccelerationLimiterConstants.maximumAcceleration);
        // SmartDashboard.putNumber("Goal bearing", goalBearing);

        //Both joysticks assumes the right to be bearing 0 and then works clockwise from there. To have bearing 0 be in front, the bearing
        //has to be moved back by 90 degrees/ 1/2 PI
        //If right joystick is not being moved retain previous bearing
        if (Math.hypot(xboxController.getRightY(), xboxController.getRightX())>  0.9) {
            joystickTurnBearing = Math.atan2(xboxController.getRightY(), xboxController.getRightX()) + Math.PI/2;
        }
        // SmartDashboard.putNumber("Turn: Right Joystick bearing", joystickTurnBearing);
        //error tolerance of 2 degrees
        if (Math.abs(joystickTurnBearing-goalBearing)>Math.PI/180*FieldOrientedDriveConstants.bearingTolerance){
            goalBearing = -joystickTurnBearing;
            bearingPIDController.reset();
            bearingPIDController.setSetpoint(goalBearing);
        }
        robotBearing = driveSubsystem.getGyroAngle();
        if (robotBearing<0){
            //converting to within range 0 to 360 degrees
            robotBearing+=360;
        }
        //converting to radians
        robotBearing = robotBearing / 180 * Math.PI;
        // SmartDashboard.putNumber("Robot bearing", robotBearing);
        //it looks cooked but that's because the controller is mapped kinda funny
        joystickMoveBearing = Math.atan2(xboxController.getLeftX(), -xboxController.getLeftY());
        // SmartDashboard.putNumber("Drive: Left joystick bearing", joystickMoveBearing);
        //gyro measures angles anticlockwise.
        joystickMoveBearing=joystickMoveBearing+robotBearing-2*Math.PI;

        // SmartDashboard.putNumber("Drive: Robot Relative bearing", joystickMoveBearing);
        joystickMoveMagnitude = Math.pow(Math.pow(xboxController.getLeftX(), 2) + Math.pow(xboxController.getLeftY(), 2), 0.5);
        // SmartDashboard.putNumber("Drive: Left joystick magnitude", joystickMoveMagnitude);

        xSpeed = joystickMoveMagnitude * Math.cos(joystickMoveBearing) * (slowModeActive ? TestingConstants.maximumSpeedReduced : TestingConstants.maximumSpeed);
        if(xSpeed>previousXSpeed+maximumAcceleration){
            xSpeed=previousXSpeed+maximumAcceleration;
        }
        else if (xSpeed<previousXSpeed- maximumAcceleration){
            xSpeed=previousXSpeed- maximumAcceleration;
        }
        previousXSpeed=xSpeed;
        // SmartDashboard.putNumber("xSpeed", xSpeed);

        ySpeed = joystickMoveMagnitude * Math.sin(joystickMoveBearing) * (slowModeActive ? TestingConstants.maximumSpeedReduced : TestingConstants.maximumSpeed);
        if(ySpeed>previousYSpeed+maximumAcceleration){
            ySpeed=previousYSpeed+ maximumAcceleration;
        }
        else if (ySpeed<previousYSpeed-maximumAcceleration){
            ySpeed=previousYSpeed-maximumAcceleration;
        }
        previousYSpeed=ySpeed;
        // SmartDashboard.putNumber("ySpeed", ySpeed);

        // rotSpeed = 0;
        rotSpeed = bearingPIDController.calculate(robotBearing) * TestingConstants.maximumRotationSpeed;
        // SmartDashboard.putNumber("rotSpeed", rotSpeed);
        

        // NOTE: both of these *may* work, this is still a topic of debate
        
        // if there is STILL some dissent regarding which mode to use, refer to Constants.java
        if (ControlConstants.controlMode == RightJoystickModes.DatisElla) {
            // Datis and Ella prefers this
            driveSubsystem.drive(xSpeed, -ySpeed, rotSpeed, false);
        } else if (ControlConstants.controlMode == RightJoystickModes.WilliamSam) {
            // William and Sam prefers this
            driveSubsystem.drive(
                xSpeed,
                -ySpeed,
                -xboxController.getRightX() * (
                    slowModeActive ?
                    TestingConstants.reducedRotationSpeedRobotOriented :
                    TestingConstants.maximumRotationSpeedRobotOriented
                ),
                false
            );
        }

        
        // Emergency stop button (Not needed)
        // if(xboxController.rightBumper().getAsBoolean()){
        //     driveSubsystem.drive(0, 0, 0, false);
        // } else{
            
        // }
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0.0, 0.0, 0.0, false);
    }
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}