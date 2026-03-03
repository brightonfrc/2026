// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  // Deploy motor – extends and retracts the intake mechanism
  private final SparkMax m_deployMotor =
      new SparkMax(IntakeConstants.kDeployMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder m_deployEncoder = m_deployMotor.getEncoder();

  // Intake spinner motor – spins the rollers to collect/eject game pieces
  private final SparkMax m_intakeMotor =
      new SparkMax(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);

  // Limit switches
  // (DigitalInput reads false when pressed on most switches)
  private final DigitalInput m_extendLimitSwitch =
      new DigitalInput(IntakeConstants.kExtendLimitSwitchPort);
  private final DigitalInput m_retractLimitSwitch =
      new DigitalInput(IntakeConstants.kRetractLimitSwitchPort);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_deployEncoder.setPosition(0.0);
  }

  // --- Sensor helpers ---

  /** @return true when the extension limit switch is activated */
  public boolean isFullyExtended() {
    return !m_extendLimitSwitch.get();
  }

  /** @return true when the retraction limit switch is activated */
  public boolean isFullyRetracted() {
    return !m_retractLimitSwitch.get();
  }

  /** @return current deploy encoder position (motor rotations) */
  public double getDeployPosition() {
    return m_deployEncoder.getPosition();
  }

  // --- Command factories ---

  /**
   * Extends the intake at low power until the extension
   * limit switch is pressed OR the encoder exceeds the
   * soft limit.
   *
   * @return a command that extends the intake
   */
  public Command extendIntakeCommand() {
    return run(() -> {
      m_deployMotor.set(IntakeConstants.kDeployPower);
    }).until(() ->
        isFullyExtended()
            || getDeployPosition()
                >= IntakeConstants.kExtendEncoderLimit
    ).finallyDo(interrupted -> m_deployMotor.set(0.0));
  }

  /**
   * Retracts the intake at low power (reversed) until
   * the retraction limit switch is pressed OR the encoder
   * drops below the soft limit.
   *
   * @return a command that retracts the intake
   */
  public Command retractIntakeCommand() {
    return run(() -> {
      m_deployMotor.set(-IntakeConstants.kDeployPower);
    }).until(() ->
        isFullyRetracted()
            || getDeployPosition()
                <= IntakeConstants.kRetractEncoderLimit
    ).finallyDo(interrupted -> m_deployMotor.set(0.0));
  }

  /**
   * Spins the intake rollers forwards at max power.
   *
   * @return a command that runs intake forwards
   */
  public Command intakeForwardsCommand() {
    return run(() -> {
      m_intakeMotor.set(IntakeConstants.kIntakeSpinPower);
    }).finallyDo(interrupted -> m_intakeMotor.set(0.0));
  }

  /**
   * Spins the intake rollers backwards at max power.
   *
   * @return a command that runs intake backwards
   */
  public Command intakeBackwardsCommand() {
    return run(() -> {
      m_intakeMotor.set(-IntakeConstants.kIntakeSpinPower);
    }).finallyDo(interrupted -> m_intakeMotor.set(0.0));
  }

  /**
   * Stops all intake motors immediately.
   *
   * @return a command that stops the intake
   */
  public Command stopIntakeCommand() {
    return runOnce(() -> {
      m_deployMotor.set(0.0);
      m_intakeMotor.set(0.0);
    });
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake/ExtendSwitch", isFullyExtended());
    SmartDashboard.putBoolean("Intake/RetractSwitch", isFullyRetracted());
    SmartDashboard.putNumber("Intake/DeployPosition", getDeployPosition());
  }
}
