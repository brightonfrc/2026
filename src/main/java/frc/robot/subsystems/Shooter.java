package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final SparkMax shooterMotor = new SparkMax(1, MotorType.kBrushless);

    private int mode = 0;
    public Command cycleModeCommand() {

        return Commands.runOnce(
            () -> {

                mode++;

                if (mode > 2) { 
                    mode = 0;
                }
                if (mode == 0) {
                    shooterMotor.set(0);
                }
                else if (mode == 1) {
                    shooterMotor.set(0.5);
                }
                else if (mode == 2) {
                    shooterMotor.set(1.0);
                }

            }
        );
    }
}