// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class TransferSubsystem extends SubsystemBase {
  public static final double BELT_SPEED = 1.0;
  public static final double INDEXER_SPEED = 1.0;

  public final SparkMax beltMotor, indexerMotor;

  private boolean isBeltRunning = false;
  private boolean isIndexerRunning = false;

  /** Creates a new TransferSubsystem. */
  public TransferSubsystem() {
    this.beltMotor = new SparkMax(13, MotorType.kBrushless);
    this.indexerMotor = new SparkMax(14, MotorType.kBrushless);
  }

  public Command toggleBeltForwards() {
      return run(
        () -> {
          if (isBeltRunning) {
            beltMotor.set(0);
          } else {
            beltMotor.set(BELT_SPEED);
          }

          isBeltRunning = !isBeltRunning;
        });
  }

  public Command toggleBeltBackwards() {
      return run(
        () -> {
          if (isBeltRunning) {
            beltMotor.set(0);
          } else {
            beltMotor.set(-BELT_SPEED);
          }

          isBeltRunning = !isBeltRunning;
        });
  }
  
  public Command toggleIndexer() {
    return run(
      () -> {
        if (isIndexerRunning) {
          indexerMotor.set(0);
        } else {
          indexerMotor.set(INDEXER_SPEED);
        }

        isIndexerRunning = !isIndexerRunning;
      });
  }
}
