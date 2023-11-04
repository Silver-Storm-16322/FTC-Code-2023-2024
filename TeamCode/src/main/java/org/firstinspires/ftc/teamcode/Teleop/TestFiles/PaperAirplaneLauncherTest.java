package org.firstinspires.ftc.teamcode.Teleop.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class PaperAirplaneLauncherTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo airplaneLauncher = hardwareMap.get(Servo.class, "paperAirplaneLauncher");
        airplaneLauncher.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                airplaneLauncher.setPosition(1);
            }

            if (gamepad1.b) {
                airplaneLauncher.setPosition(0);
            }
        }
    }

}
