package com.spamrobotics.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class TimedSubsystem extends SubsystemBase {
    
    private double loopTimeMS;

    @Override
    public void periodic() {
        double start = Timer.getTimestamp();
        timedPeriodic();
        loopTimeMS = (Timer.getTimestamp() - start) * 1000; // Convert to milliseconds
    }

    public double getLoopTimeMS() {
        return loopTimeMS;
    }

    public abstract void timedPeriodic();
}
