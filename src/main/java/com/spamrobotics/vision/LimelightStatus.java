package com.spamrobotics.vision;

import com.spamrobotics.util.LimelightHelpers;

import edu.wpi.first.wpilibj.Timer;

public class LimelightStatus implements CameraStatus {

    private final String limelightName;

    private double limelightHeartbeat = 0;
    private double lastHeartbeatTime = 0;
    private boolean limelightConnected = false;

    public LimelightStatus(String limelightName) {
        this.limelightName = limelightName;
    }

    @Override
    public void update() {
        double newHeartbeat = LimelightHelpers.getLimelightNTDouble(limelightName, "hb");
        if (newHeartbeat > limelightHeartbeat) {
            limelightConnected = true;
            limelightHeartbeat = newHeartbeat;
            lastHeartbeatTime = Timer.getFPGATimestamp();
        } else if (Timer.getFPGATimestamp() - lastHeartbeatTime >= 1) {
            limelightConnected = false;
        }
    }

    @Override
    public boolean isConnected() {
        return limelightConnected;
    }
}
