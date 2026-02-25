// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.OIConstants;
import frc.robot.commands.CoralStationAlign;
import frc.robot.commands.FieldOrientedDrive;
// import frc.robot.subsystems.ColourSensor;
import frc.robot.subsystems.DriveSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.StopRobot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.AprilTagPoseEstimator;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem m_driveSubsystem= new DriveSubsystem();
  private final AprilTagPoseEstimator m_poseEstimator = new AprilTagPoseEstimator();

  private boolean active = false;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
  new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandPS4Controller m_manualLiftController= new CommandPS4Controller(OIConstants.kManualLiftControllerPort);
  private final FieldOrientedDrive fieldOrientedDrive= new FieldOrientedDrive(m_driveSubsystem, m_driverController);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_driveSubsystem.setDefaultCommand(fieldOrientedDrive);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

   //public static boolean aliningWithCoral = false;

   public static boolean fieldRelative = true;

  private void configureBindings() {
    //R2 and L2 don't work
    if (m_driverController.x().getAsBoolean()){
      System.out.println("Gyro reset");
    }
    m_driverController.x().onTrue( // Reset gyro whenever necessary
      new InstantCommand(() -> m_driveSubsystem.resetGyro(), m_driveSubsystem)
    );
    m_driverController.leftBumper().onTrue(new CoralStationAlign(m_driveSubsystem, m_driverController));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    StopRobot stop = new StopRobot(m_driveSubsystem);
    //annoyingly, you can't reuse the same command in a Commands.sequence
    // StopRobot stop2 = new StopRobot(m_driveSubsystem);
    // StopRobot stop3 = new StopRobot(m_driveSubsystem);
    // StopRobot stop4 = new StopRobot(m_driveSubsystem);

    return Commands.sequence(
      new PathPlannerAuto("Auto"),
      stop
    );
    }
  public void SetUpDefaultCommand(){
    m_driveSubsystem.setDefaultCommand(fieldOrientedDrive);
  }

  public void resetBearings(){  
    m_driveSubsystem.resetOdometry(m_driveSubsystem.getPose());
    m_driveSubsystem.resetGyro();
  }

  public void resetGyro(){
    m_driveSubsystem.resetGyro();
  }
  public Pose2d getPose(){
    return m_driveSubsystem.getPose();
    // SmartDashboard.putString("Auto", "AprilTag Alignment");
    // return new AprilTagAlignment(m_driveSubsystem, new AprilTagPoseEstimator(), 3, 0.5);
  }

  // TODO: Delete
  public void printPose() {
    Optional<Transform3d> opt = m_poseEstimator.getRobotToSeenTag();
    if(opt.isPresent()) {
      Transform3d r2t = opt.get();
      // SmartDashboard.putString("robot2tag", r2t.getTranslation().toString() + "Rotation3d(yaw="+r2t.getRotation().getZ()+", pitch="+r2t.getRotation().getY()+", roll="+r2t.getX()+")");
      SmartDashboard.putNumber("robot2tag/t/x", r2t.getTranslation().getX());
      SmartDashboard.putNumber("robot2tag/t/y", r2t.getTranslation().getY());
      SmartDashboard.putNumber("robot2tag/r/yaw", r2t.getRotation().getZ());
    }
  }

}

