package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    private SparkMax ShooterMotor;
    private int ShooterMode;
    public ShooterSubsystem(SparkMax ShooterMotor){
        this.ShooterMotor = ShooterMotor;
    }
    public Command toggle(){
        return runOnce(() -> {
            ShooterMode = (ShooterMode + 1) % 3;
        });
    }
    public Command Shoot(){
        return runOnce(
            () -> {
                if(ShooterMode == 0){
                    ShooterMotor.set(0);
                }
                else if(ShooterMode == 1){
                    ShooterMotor.set(0.3);
                }
                else if(ShooterMode == 2){
                    ShooterMotor.set(1);
                }
            }
        );
    }
}
