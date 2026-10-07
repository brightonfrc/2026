public class IntakeSubsystem {
    private SparkMax intakeMotor;

    public IntakeSubsystem(SparkMax intakeMotor) {
        this.intakeMotor = intakeMotor;
    }

    public Command startIntake() {
        return runOnce(
            () -> {
                intakeMotor.set(1);
            }
        )
    }

    public Command stopIntake() {
        return runOnce(
            () -> {
                intakeMotor.set(0);
            }
        )
    }
}
