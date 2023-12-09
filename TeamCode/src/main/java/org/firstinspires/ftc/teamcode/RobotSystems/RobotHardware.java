package org.firstinspires.ftc.teamcode.RobotSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.Manipulator;
import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.PaperAirplaneLauncher;
import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.Vision;

public class RobotHardware {

    // Declare Hardware Variables.
    private final LinearOpMode myOpMode;
    public DriveTrain driveTrain = null;
    public Manipulator manipulator = null;
    public PaperAirplaneLauncher paperAirplaneLauncher = null;
    public Vision vision = null;

    public RobotHardware (LinearOpMode opMode) { this.myOpMode = opMode; }

    public void init() {

        // Initialize robot subsystems
        driveTrain = new DriveTrain(myOpMode);
        driveTrain.init();

        manipulator = new Manipulator();
        manipulator.init(myOpMode.hardwareMap, myOpMode.telemetry);

        paperAirplaneLauncher = new PaperAirplaneLauncher(myOpMode);
        paperAirplaneLauncher.init();

        vision = new Vision(myOpMode);
        vision.init();

        // Tell the user that the hardware has been initialized.
        myOpMode.telemetry.addData("->", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
}
