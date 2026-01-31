// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.ColourSensorConstants;


import frc.robot.Constants.AutonomousNavConstants;
import frc.robot.Constants.ChoreoConstants;
import frc.robot.Constants.CoralStationAlignConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.WinchConstants;
import frc.robot.Constants.LiftConstants.Height;
import frc.robot.Constants.AprilTagAlignmentConstants;
import frc.robot.commands.AprilTagAlignment;
import frc.robot.commands.Autos;
import frc.robot.commands.CoralStationAlign;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FieldOrientedDrive;
// import frc.robot.subsystems.ColourSensor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.MoveToPoint;
import frc.robot.commands.StopRobot;
import frc.robot.loggers.GenericLogger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.AprilTagPoseEstimator;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.loggers.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final AprilTagPoseEstimator m_poseEstimator = new AprilTagPoseEstimator();

  private final GenericLogger logger;

  private boolean active = false;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
  new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandPS4Controller m_manualLiftController = new CommandPS4Controller(OIConstants.kManualLiftControllerPort);
  private final FieldOrientedDrive fieldOrientedDrive= new FieldOrientedDrive(m_driveSubsystem, m_driverController);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(GenericLogger logger) {
    // Configure the trigger bindings
    m_driveSubsystem.setDefaultCommand(fieldOrientedDrive);

    // we need to assign the logger to every single child.
    // this means we use the same logger *everywhere*, so we pass it down.
    // it is not static, as in the future we may add a logging system that is stateful, so *JUST IN CASE*
    // As logging is optional, we have this method to assign it rather than pass into constructor
    //
    // As some commands/susbsystems aren't used, comments are used to replace them,
    // to serve as a reminder to do this for all future commands
    m_driveSubsystem.assignLogger(logger);
    m_poseEstimator.assignLogger(logger);
    /* assign logger to AprilTagAlignment */
    /* assign logger to CoralStationAlign: this is done lower down in configureBindings() */
    /* assign logger to MoveToPoint */
    fieldOrientedDrive.assignLogger(logger);
    this.logger = logger;
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

    // TODO: move creation of CoralStationAlign outside of a method
    CoralStationAlign coral_a = new CoralStationAlign(m_driveSubsystem, m_driverController);
    coral_a.assignLogger(logger);
    m_driverController.leftBumper().onTrue(coral_a);
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
    // logger.logString("Auto", "AprilTag Alignment");
    // return new AprilTagAlignment(m_driveSubsystem, new AprilTagPoseEstimator(), 3, 0.5);
  }

  // TODO: Delete
  public void printPose() {
    Optional<Transform3d> opt = m_poseEstimator.getRobotToSeenTag();
    if(opt.isPresent()) {
      Transform3d r2t = opt.get();
      logger.logString("robot2tag", r2t.getTranslation().toString() + "Rotation3d(yaw="+r2t.getRotation().getZ()+", pitch="+r2t.getRotation().getY()+", roll="+r2t.getX()+")");
      logger.logDouble("robot2tag/t/x", r2t.getTranslation().getX());
      logger.logDouble("robot2tag/t/y", r2t.getTranslation().getY());
      logger.logDouble("robot2tag/r/yaw", r2t.getRotation().getZ());
    }
  }

}

