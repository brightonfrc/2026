package frc.robot.loggers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardLogger implements GenericLogger {
    public SmartDashboardLogger() {
        /* this constructor does nothing, because all of SmartDashboard's methods are static */
    }

    @Override
    public void log(String label, double data) {
        SmartDashboard.putNumber(label, data);
    }

    @Override
    public void log(String label, long data) {
        SmartDashboard.putNumber(label, (double) data);
    }

    @Override
    public void log(String label, boolean data) {
        SmartDashboard.putBoolean(label, data);
    }
    
    @Override
    public void log(String label, String data) {
        SmartDashboard.putString(label, data);
    }

    @Override
    public void echo(String data) {
        SmartDashboard.putString("<none>", data);
        GenericLogger.printLog(data);
    }
}
