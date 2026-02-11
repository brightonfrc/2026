// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


// two sparkmax motor controlers, one for intake and one for accelerator
public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_intakeMotor =
      new SparkMax(IntakeConstants.intakeCanID, MotorType.kBrushless);
  private final SparkMax m_acceleratorMotor =
      new SparkMax(IntakeConstants.acceleratorCanID, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  public void setIntake(double power) {
    m_intakeMotor.set(power);
  }

  public void setAccelerator(double power) {
    m_acceleratorMotor.set(power);
  }

  public void stop() {
    m_intakeMotor.set(0.0);
    m_acceleratorMotor.set(0.0);
  }

  /**
   * Command factory: runs both intake and accelerator to pull balls
   *
   * @return a command that runs the intake
   */
  public Command intakeCommand() {
    return run(() -> {
      setIntake(IntakeConstants.intakePower);
      setAccelerator(IntakeConstants.acceleratorPower);
      //finally do stops the motor when commands end
    }).finallyDo(interrupted -> stop());
  }

  /**
   * Command factory: reverses both motors to eject game pieces.
   *
   * @return a command that runs the outtake
   */
  public Command outtakeCommand() {
    return run(() -> {
      setIntake(IntakeConstants.outtakePower);
      setAccelerator(-IntakeConstants.acceleratorPower); 
    }).finallyDo(interrupted -> stop());
  }

  /**
   * Command factory: runs only the accelerator at full power to shoot.
   *
   * @return a command that shoots
   */
  public Command shootCommand() {
    return run(() -> {
      setAccelerator(IntakeConstants.acceleratorPower);
    }).finallyDo(interrupted -> stop());
  }

  /**
   * Command factory: stops all motors.
   *
   * @return a command that stops the intake
   */
  public Command stopCommand() {
    return runOnce(() -> stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
