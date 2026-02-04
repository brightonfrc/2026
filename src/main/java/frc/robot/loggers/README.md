# GenericLogger interface
The `GenericLogger` interface is an interface to abstract all logging of data behind.

This is to make using multiple logging backends much easier, and to ease the switchover to AdvantageScope.

## Implementations
Below are the functions that all `GenericLogger`s must implement.

### Labelled logging
All functions beginning `log` take a label and data as an argument, which should log a simple number to its respective backend as a datapoint under the given label.
There are many different datatypes of `data` accepted, but all should do the same thing.

Here is a list of all of them below:
`void log(String label, double data);`
`void log(String label, long data);`
`void log(String label, boolean data);`
`void log(String label, String data);`

### Miscellaneous
`void echo(String data);` just prints out a string. This is useful for debugging messages, such as having reached certain points of code (e.g. `"Started Teleop"`). 

## How it used to work
This is replacing older code that involved manually calling SmartDashboard. This abstracts away the explicit calls to SmartDashboard, ensuring that once it becomes deprecated and ~~we begin to use `AdvantageScope`, the switchover will be very easy.~~ ***N.B.:* We are now using Glass, forget we ever talked about AdvantageScope**

## Usage in source amongst subsystems and commands
As `GenericLogger` classes may be stateful, instances of them are passed to each subsystem and command.

The logger is initialised in the `Robot` class, whence it is passed into the constructor of the `RobotContainer` class, with which it calls the `setLogger` method of all "loggable" commands.
Any command or subsystem that requires it extends one of the "loggable" `CommandWithLogger` or `SubsystemBaseWithLogger` classes, rather than the simple `Command` and `Subsystem` classes; these provide a `logger` property to children, and a `setLogger` method, which `RobotContainer` uses.


## Types that exist

| Logger type | Purpose |
|-|-|
| `BlankLogger` | As a placeholder logger in the loggable classes. Warns to stdout if logged with. (q.v. for more info) |
| `SmartDashboardLogger` | Very simple abstraction over SmartDashboard. Eventually, will no longer be used |

