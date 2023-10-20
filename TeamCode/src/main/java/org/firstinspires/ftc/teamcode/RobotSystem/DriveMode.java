package org.firstinspires.ftc.teamcode.RobotSystem;

public enum DriveMode {
    DEFAULT_DRIVE(1),
    PRECISE_DRIVE (0.25),
    SENSITIVE_DRIVE (10);

    private final double speedMultiplier;
    DriveMode(double speedMultiplier) {
        this.speedMultiplier = speedMultiplier;
    }

    double speedMultiplier() { return this.speedMultiplier; }
}

