package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangSubsystem extends SubsystemBase {
    // TODO: tune these
    public static final double HANG_MOTOR_SPEED = 0.2;
    public static final int HANG_DISTANCE = 200;
    public static final int UNHANG_DISTANCE = 10;

    private final SparkMax hangMotor;
    private final Encoder hangEncoder;

    private boolean isHanging = false; // true = hang, false = unhang

    public HangSubsystem() {
        // TODO: put device id in a constants class once all the subsystems have been merged
        hangMotor = new SparkMax(15, MotorType.kBrushless);

        // set break mode
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        hangMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        hangEncoder = new Encoder(0, 1);
        hangEncoder.reset();
    }

    @Override
    public void periodic() {
        if (isHanging && hangEncoder.getDistance() < HANG_DISTANCE) {
            hangMotor.set(HANG_MOTOR_SPEED);
        } else if (!isHanging && hangEncoder.getDistance() > UNHANG_DISTANCE) {
            hangMotor.set(-HANG_MOTOR_SPEED);
        } else {
            hangMotor.set(0);
        }
    }

    public Command winch() {
        return run(
            () -> {
                isHanging = true;
            });
    }

    public Command unwinch() {
        return run(
            () -> {
                isHanging = false;
            });
    }
}
