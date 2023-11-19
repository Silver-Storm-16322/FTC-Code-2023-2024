package org.firstinspires.ftc.teamcode.RobotSystems.Commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotSystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutoParams;

public abstract class Commands {

    protected RobotHardware robot;
    protected Telemetry telemetry;
    protected AutoParams autoParams;

    /**
     * Setup all of the values that every command will be using.
     *
     * @param robot The robot's hardware. Allows us to access of of it's available subsystems.
     * @param telemetry Allows us to convey info to the driver.
     */
    public void init(RobotHardware robot, Telemetry telemetry, AutoParams autoParams) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.autoParams = autoParams;
    }

    public abstract void run();
    public abstract boolean isCommandFinished();
}
