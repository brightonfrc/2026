package frc.robot.loggers;

/**
 * This is a generic interface for a logger. 
 * Basically, we have no idea of what the hell kind of logger we're using, so we have a generic interface,
 * so that any forseeable refactors are not hell
 */
public interface GenericLogger {
    void logDouble(String label, double data);
    void logInteger(String label, long data);
    void logBool(String label, boolean data);
    void logString(String label, String data);
}