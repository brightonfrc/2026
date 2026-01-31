package frc.robot.loggers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardLogger implements GenericLogger {
    public SmartDashboardLogger() {
        /* this constructor does nothing, because all of SmartDashboard's methods are static */
    }

    public void logDouble(String label, double data) {
        SmartDashboard.putNumber(label, data);
    }

    public void logInteger(String label, long data) {
        SmartDashboard.putNumber(label, (double) data);
    }

    public void logBool(String label, boolean data) {
        SmartDashboard.putBoolean(label, data);
    }
    
    public void logString(String label, String data) {
        SmartDashboard.putString(label, data);
    }
}
