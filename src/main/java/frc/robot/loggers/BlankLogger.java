package frc.robot.loggers;

/**
 * Do nothing.
 * A default logger in every single class: is an important design decision that LOGGING IS OPTIONAL.
 */
public class BlankLogger implements GenericLogger {
    public BlankLogger() { }

    /**
     * BlankLogger shouldn't really be used at all: this just warns the user.
     * @param message
     */
    private void warn(String message) {
        GenericLogger.printLog("BlankLogger: " + message + "\n\tPlease check code for uninitialised loggers.");
    }
    private void warn() {
        this.warn("Attempted to log data.");
    }

    // lots of methods, all of which we just set to warn the user
    @Override public void log(String label, double data) { this.warn(); }
    @Override public void log(String label, long data)  { this.warn(); }
    @Override public void log(String label, boolean data)  { this.warn(); }
    @Override public void log(String label, String data) { this.warn(); }

    // this one we actually provide a data, so it's not completely useless
    @Override public void echo(String data) { this.warn(data); }
}
