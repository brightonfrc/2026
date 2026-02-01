package frc.robot.loggers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardLogger implements GenericLogger {
    public SmartDashboardLogger() {
        /* this constructor does nothing, because all of SmartDashboard's methods are static */
    }

    @Override
    public void logDouble(String label, double data) {
        SmartDashboard.putNumber(label, data);
    }

    @Override
    public void logInteger(String label, long data) {
        SmartDashboard.putNumber(label, (double) data);
    }

    @Override
    public void logBool(String label, boolean data) {
        SmartDashboard.putBoolean(label, data);
    }
    
    @Override
    public void logString(String label, String data) {
        SmartDashboard.putString(label, data);
    }

    @Override
    public void echo(String data) {
        SmartDashboard.putString("<none>", data);
        GenericLogger.printLog(data);
    }
}
