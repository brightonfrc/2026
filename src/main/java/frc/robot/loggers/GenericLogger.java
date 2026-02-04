package frc.robot.loggers;

/**
 * This is a generic interface for a logger. 
 * Basically, we have no idea of what kind of logger we're going to be using, so we have a generic interface,
 * so that any forseeable refactors are not hell
 */
public interface GenericLogger {
    void log(String label, double data);
    void log(String label, long data);
    void log(String label, boolean data);
    void log(String label, String data);
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