package frc.robot.loggers;

/**
 * This is a generic interface for a logger. 
 * Basically, we have no idea of what kind of logger we're going to be using, so we have a generic interface,
 * so that any forseeable refactors are not hell
 */
public interface GenericLogger {
    void logDouble(String label, double data);
    void logInteger(String label, long data);
    void logBool(String label, boolean data);
    void logString(String label, String data);
    void echo(String data);

    /**
     * Helper method to dump data to a log.
     * Reason for existance is that in forseeable future, may not just be console.
     * @param data
     */
    static void printLog(String msg) {
        System.out.println("Logger: " + msg);
    }
}