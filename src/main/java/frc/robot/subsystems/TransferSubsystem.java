// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;

public class TransferSubsystem extends SubsystemBase {
  public SparkMax motor1;
  public SparkMax motor2;
  public SparkMax motor3;
  public SparkMax motor4;

  /** Creates a new TransferSubsystem. */
  public TransferSubsystem(SparkMax motor1, SparkMax motor2, SparkMax motor3, SparkMax motor4) {
    this.motor1 = motor1;
    this.motor2 = motor2;
    this.motor3 = motor3;
    this.motor4 = motor4;
  }

  public void setPower(double power) {
    this.motor1.set(power);
    this.motor2.set(power);
    this.motor3.set(power);
    this.motor4.set(power);
  }
}
