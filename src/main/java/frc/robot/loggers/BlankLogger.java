package frc.robot.loggers;

/**
 * Do nothing.
 * A default logger in every single class: is an important design decision that LOGGING IS OPTIONAL.
 */
public class BlankLogger implements GenericLogger {
    public BlankLogger() { }

    public void logDouble(String label, double data) { }

    public void logInteger(String label, long data) { }

    public void logBool(String label, boolean data) { }
    
    public void logString(String label, String data) { }
}
