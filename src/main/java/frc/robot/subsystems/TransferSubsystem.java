// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;

public class TransferSubsystem extends SubsystemBase {
  public static final double BELT_SPEED = 1.0;
  public static final double INDEXER_SPEED = 1.0;

  public final SparkMax beltMotor, indexerMotor;

  private boolean isBeltRunning = false;
  private boolean isIndexerRunning = false;

  /** Creates a new TransferSubsystem. */
  public TransferSubsystem(SparkMax beltMotor, SparkMax indexerMotor) {
    this.beltMotor = beltMotor;
    this.indexerMotor = indexerMotor;
  }

  public Command toggleBeltForwards() {
      return runOnce(
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
      return runOnce(
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
    return runOnce(
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
