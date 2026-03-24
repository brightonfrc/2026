// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Constants.LiftConstants.Height;

// Holds robot-wide numerical or boolean constants.
public final class Constants {
    public static class AutonomousNavConstants {
        public static double TRANSLATION_KP = 0.1;
        public static double TRANSLATION_KI = 0;
        public static double TRANSLATION_KD = 0;
        public static double TRANSLATION_TOLERANCE = 0.02;
        public static double ROTATION_KP = 0.25;
        public static double ROTATION_KD = 0.005;
        public static double ROTATION_KI = 0.01;
        // 0.5 degrees of tolerance
        public static double ROTATION_TOLERANCE = Math.PI / 360;

        public static Height SCORE_HEIGHT = Height.L4;

        // TODO: remember to change this
        public static StartingPosition START_POS = StartingPosition.Taxi;

        public static enum StartingPosition {
            Left,
            Right,
            Middle,
            Taxi,
            Testing
        }

        // the gyro is inverted, so 90 degrees is to the left
        // bearing of robot when first Choreo Path Ends.
        // TODO: ASSIGN THESE VALUES AS NEEDED
        public static double END_ROT_ONE = switch (START_POS) {
            case Left -> 300; // Should be 60 to the right, but just don't want it to go out of bounds...
            case Right -> 60;
            case Middle -> 0;
            default -> 0;
        };
    }

    public static class LiftConstants {
        // for some reason at 0.04 the robot goes insane
        public static final double MAXIMUM_POWER_CHANGE = 0.05;
        public static final double MAXIMUM_TILT = 10;
        public static final double ANGLE_AT_PEAK_HEIGHT = 278.2;
        public static final int ENCODER_CHANNEL = 0;
        public static final int LIFT_NEO_CAN_ID = 11;
        public static final int REVERSED_LIFT_NEO_CAN_ID = 12;
        // to ensure the arm doesn't tear itself apart.
        public static final double MAX_POWER = 0.7;

        // angles required
        public static final double[] DESIRED_LIFT_ANGLE = new double[] {
            // do note that ground height isn't actually at 0, because arm has minimum height
            Math.toRadians(10), // ground intake (placeholder)
            Math.toRadians(18), // L1
            Math.toRadians(24), // L2 is 16 degrees below L3, so test L3 first
            Math.toRadians(44), // L3
            Math.toRadians(87), // L4
            Math.toRadians(40), // Algae2
            Math.toRadians(68), // Algae3 (placeholder)
            Math.toRadians(26), // CoralStation
            Math.toRadians(35), // StartingConfig
            Math.toRadians(20), // hang start
            Math.toRadians(18)  // Internal Stow
        };

        public static enum Height {
            Ground,
            L1,
            L2,
            L3,
            L4,
            StartingConfig,
            Algae2, // algae between L2 and L3
            Algae3, // algae between L3 and L4
            // TODO: NUKE
            CoralStation,
            HangStart,
            InternalStow
        }

        public static final double ARM_LENGTH = 27.4 * 0.0254;
        public static final double P_LIFT = 0.29; // 0.6
        public static final double I_LIFT = 0.0; // 0.03
        public static final double D_LIFT = 0;
        public static final double ANGLE_TOLERANCE = Math.PI / 180;

        // remember to adjust as more components are added
        public static final double WEIGHT_MOMENT_OFFSET_FACTOR = 0.0;

        public static final double LIFT_FALLING_POWER = -0.4; // -0.25
        public static final double L4_OUTTAKE_ANGLE = Math.toRadians(75);
        public static final double L4_OUTTAKE_END = Math.toRadians(50);
        public static final double P_HANG = 1.2;
        public static final double I_HANG = 0.5;
        public static final double D_HANG = 0.0;
    }

    public static class TestingConstants {
        public static final double MAXIMUM_SPEED = 0.4;
        public static final double MAXIMUM_ROTATION_SPEED = 1;
        public static final double MAXIMUM_SPEED_REDUCED = 0.10;
        public static final double MAXIMUM_ROTATION_SPEED_ROBOT_ORIENTED = 0.5;
        public static final double REDUCED_ROTATION_SPEED_ROBOT_ORIENTED = 0.10;
    }

    public static final class FieldOrientedDriveConstants {
        public static final double FOD_P = 0.25;
        public static final double FOD_I = 0.3;
        public static final double FOD_D = 0.0;

        // Maximum rotation speed
        public static final double ROTATION_SCALAR = Math.PI;

        public static final double BEARING_TOLERANCE = 0.5;
    }

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double MAX_SPEED_METERS_PER_SECOND = 4.8;
        public static final double MAX_ANGULAR_SPEED = 4 * Math.PI; // radians per second

        public static final double DIRECTION_SLEW_RATE = 1.2; // radians per second
        public static final double MAGNITUDE_SLEW_RATE = 1.8; // percent per second (1 = 100%)
        public static final double ROTATIONAL_SLEW_RATE = 2.0; // percent per second (1 = 100%)

        // Chassis configuration
        // Distance between centers of right and left wheels on robot
        public static final double TRACK_WIDTH = 0.574;
        // Distance between front and back wheels on robot
        public static final double WHEEL_BASE = 0.574;
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
        public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
        public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
        public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int FRONT_LEFT_DRIVING_CAN_ID = 8;
        public static final int FRONT_LEFT_TURNING_CAN_ID = 1;
        public static final int FRONT_RIGHT_DRIVING_CAN_ID = 7;
        public static final int FRONT_RIGHT_TURNING_CAN_ID = 5;
        public static final int REAR_LEFT_DRIVING_CAN_ID = 4;
        public static final int REAR_LEFT_TURNING_CAN_ID = 3;
        public static final int REAR_RIGHT_DRIVING_CAN_ID = 2;
        public static final int REAR_RIGHT_TURNING_CAN_ID = 6;

        public static final boolean GYRO_REVERSED = false;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int DRIVING_MOTOR_PINION_TEETH = 13;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean TURNING_ENCODER_INVERTED = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
        public static final double WHEEL_DIAMETER_METERS = 0.0762;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
            / DRIVING_MOTOR_REDUCTION;

        public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
            / DRIVING_MOTOR_REDUCTION; // meters
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
            / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
        public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

        public static final double DRIVING_P = 0.04;
        public static final double DRIVING_I = 0;
        public static final double DRIVING_D = 0;
        public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
        public static final double DRIVING_MIN_OUTPUT = -1;
        public static final double DRIVING_MAX_OUTPUT = 1;

        public static final double TURNING_P = 1;
        public static final double TURNING_I = 0;
        public static final double TURNING_D = 0;
        public static final double TURNING_FF = 0;
        public static final double TURNING_MIN_OUTPUT = -1;
        public static final double TURNING_MAX_OUTPUT = 1;

        public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

        public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50; // amps
        public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps
    }

    public static final class NeoMotorConstants {
        public static final double FREE_SPEED_RPM = 5676;
    }
}
