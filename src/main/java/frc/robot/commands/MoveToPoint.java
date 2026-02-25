// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutonomousNavConstants;
import frc.robot.subsystems.DriveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToPoint extends Command {
  private DriveSubsystem driveSubsystem;
  private PIDController rotController;
  private double goal; 
//   public MoveToPoint(DriveSubsystem driveSubsystem, Pose2d goal) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.driveSubsystem=driveSubsystem;
//     addRequirements(driveSubsystem);
//     this.goal=goal;
//   }
  public MoveToPoint(DriveSubsystem driveSubsystem, double goal){
    this.driveSubsystem=driveSubsystem;
    this.goal=goal;
    addRequirements(driveSubsystem);
  }

  // public MoveToPoint(DriveSubsystem driveSubsystem, Pose2d goalPose2d){
  //   this.driveSubsystem=driveSubsystem;
  //   this.goal=goalPose2d;
  //   addRequirements(driveSubsystem);
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // xController=new PIDController(AutonomousNavConstants.translationkP,AutonomousNavConstants.translationkI, AutonomousNavConstants.translationkD);
    // xController.setSetpoint(goal.getX());
    // xController.setTolerance(AutonomousNavConstants.translationTolerance);
    // yController=new PIDController(AutonomousNavConstants.translationkP,AutonomousNavConstants.translationkI, AutonomousNavConstants.translationkD);
    // yController.setSetpoint(goal.getY());
    // yController.setTolerance(AutonomousNavConstants.translationTolerance);
    rotController=new PIDController(AutonomousNavConstants.rotationkP, AutonomousNavConstants.rotationkI, AutonomousNavConstants.rotationkD);
    rotController.setTolerance(AutonomousNavConstants.rotationTolerance);
    rotController.enableContinuousInput(0, 2*Math.PI);
    double radiansSetpoint=goal;
    //convert to range 0 to 2 PI
    if (radiansSetpoint<0){
      radiansSetpoint+=2*Math.PI;
    }
    rotController.setSetpoint(radiansSetpoint);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double robotBearing = driveSubsystem.getGyroAngle();
    if (robotBearing<0){
      //converting to within range 0 to 360 degrees
      robotBearing+=360;
    }
    // SmartDashboard.putNumber("Pose/x", driveSubsystem.getPose().getX());
    // SmartDashboard.putNumber("Pose/y", driveSubsystem.getPose().getY());
    SmartDashboard.putBoolean("MoveToPointActive", true);

    //convert to radians
    robotBearing=Math.toRadians(robotBearing);
    SmartDashboard.putNumber("Bearing", robotBearing);
    double rotSpeed=rotController.calculate(robotBearing);
    // double xSpeed=xController.calculate(-driveSubsystem.getPose().getX());
    // double ySpeed=yController.calculate(-driveSubsystem.getPose().getY());
    // SmartDashboard.putNumber("Speed/x", xSpeed);
    // SmartDashboard.putNumber("Speed/y", ySpeed);
    //rotation works
    // SmartDashboard.putNumber("Speed/rot", rotSpeed);
    // driveSubsystem.drive(xSpeed, 0, 0, false);
    // if (xController.atSetpoint()&&yController.atSetpoint()){
    //   driveSubsystem.drive(0, 0, rotSpeed, false);
    // }
    driveSubsystem.drive(0, 0, rotSpeed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false);
    SmartDashboard.putBoolean("MoveToPointActive", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return xController.atSetpoint()&&yController.atSetpoint()&&rotController.atSetpoint();
    return rotController.atSetpoint();
  }
}
