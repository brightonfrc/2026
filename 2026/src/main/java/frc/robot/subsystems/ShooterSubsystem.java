package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax motorTop = new SparkMax(1, MotorType.kBrushless);
    private final SparkMax motorBottom = new SparkMax(2, MotorType.kBrushless);
    private final SparkMax motorHood = new SparkMax(3, MotorType.kBrushless);

    public ShooterSubsystem() {
        
    }

    public void runTopMotor(double power) {
        motorTop.set(power);
    }

    public void runBottomMotor(double power) {
        motorBottom.set(power);
    }

    public void setHoodPower(double power) {
        motorHood.set(power);
    }

    public void stopShooter() {
        motorTop.set(0);
        motorBottom.set(0);
        motorHood.set(0);
    }
}