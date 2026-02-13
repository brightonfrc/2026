// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class StartShooter extends Command {
    private final ShooterSubsystem shooter;
    private final double topPower;
    private final double hoodPower;
    private final Timer timer = new Timer();

    //We will calculate the parameters fromt the computer vision
    public StartShooter(ShooterSubsystem shooter, double topPower, double hoodPower) {
        this.shooter = shooter;
        this.topPower = topPower;
        this.hoodPower = hoodPower;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        
        shooter.runTopMotor(topPower);
        shooter.setHoodPower(hoodPower);
    }

    @Override
    public void execute() {
        //How much time should we wait until we turn on the bottom motor?
        if (timer.get() > 0.8) {
          //Power of the bottom motor?
            shooter.runBottomMotor(0.6);
        }
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
    }
}
