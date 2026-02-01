package frc.robot.loggers.loggables;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.loggers.BlankLogger;
import frc.robot.loggers.GenericLogger;

/**
 * Dummy Subsystem class that provides logging functionality to children.
 */
public class SubsystemBaseWithLogger extends SubsystemBase {
    protected GenericLogger logger = new BlankLogger();

    /**
     * Assign logger. refer to RobotContainer.java for the rationale behind this
     * @param logger
     */
    public void setLogger(GenericLogger logger) {
        this.logger = logger; 
    } 
}
