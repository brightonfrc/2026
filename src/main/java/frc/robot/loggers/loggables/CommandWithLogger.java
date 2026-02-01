package frc.robot.loggers.loggables;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.loggers.BlankLogger;
import frc.robot.loggers.GenericLogger;

/**
 * Dummy Command class that provides logging functionality to children.
 */
public class CommandWithLogger extends Command {
    protected GenericLogger logger = new BlankLogger();

    /**
     * Assign logger. refer to RobotContainer.java for the rationale behind this
     * @param logger
     */
    public void setLogger(GenericLogger logger) {
        this.logger = logger; 
    } 
}
