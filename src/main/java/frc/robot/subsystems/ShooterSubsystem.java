public class ShooterSubsystem {
    private SparkMax shooterMotor;

    private double low = 0.5;
    private double high = 1;

    private int currentCommand = 0;

    public ShooterSubsystem(shooterMotor) {
        this.shooterMotor = shooterMotor;
    }

    public Command lowPower() {
        return runOnce(
            () -> {
                currentCommand = 1;
                shooterMotor.set(low);
            }
        )
    }

    public Command highPower() {
        return runOnce(
            () -> {
                currentCommand = 2;
                shooterMotor.set(high);
            }
        )
    }

    public Command off() {
        return runOnce(
            () -> {
                currentCommand = 0;
                shooterMotor.set(0);
            }
        )
    }

    public Command toggle() {
        () -> {
            if (currentCommand == 0) {
                shooterMotor.set(low);
                currentCommand = 1;
                }
            else if (currentCommand == 1) {
                shooterMotor.set(high);
                currentCommand = 2;
            }
            else {
                shooterMotor.set(0);
                currentCommand = 0;
            }
        }
    }
}