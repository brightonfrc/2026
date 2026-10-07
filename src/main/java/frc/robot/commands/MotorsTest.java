package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class MotorsTest extends Command {
    public static final int N_TESTED_MOTORS = 8;

    private static SparkMax[] motors = new SparkMax[N_TESTED_MOTORS];

    private int currentMotorId = 0;

    private final XboxController xboxController;

    private boolean wasDpadPressed = false;

    public MotorsTest(XboxController xboxController, DriveSubsystem driveSubsystem) {
        this.xboxController = xboxController;

        // N.B. indices count from 0 to 7 inclusive, can IDs count from 1 to 8 inclusive
        /*
        for (int canId = 1; canId <= 8; canId++) {
            motors[canId - 1] = new SparkMax(canId, MotorType.kBrushless);
        }
        */

        motors = driveSubsystem.getMotors();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // dpad bindings are weird: 90 is right, 270 is left, and -1 is unpressed
        int povAngle = xboxController.getPOV();

        if (!wasDpadPressed) {
            if (povAngle == 90) {
                motors[currentMotorId].set(0);

                currentMotorId++;
                if (currentMotorId >= motors.length) {
                    currentMotorId -= motors.length;
                }
            } else if (povAngle == 270) {
                motors[currentMotorId].set(0);

                currentMotorId--;
                if (currentMotorId < 0) {
                    currentMotorId += motors.length;
                }
            }
        }

        wasDpadPressed = povAngle != -1;

        double motorSpeed = -xboxController.getLeftY();
        motors[currentMotorId].set(motorSpeed);

        SmartDashboard.putNumber("Motor Id", currentMotorId);
        SmartDashboard.putNumber("Motor Speed", motorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        motors[currentMotorId].set(0);
    }
}
