package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Что нибудь", group="")
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!

        R.g(2500);
        R.delay(500);
        R.g(-2500);
        R.delay(500);
        R.g(2500);
        R.delay(500);
        R.g(-2500);
        R.delay(500);;
        R.g(2500);
        R.delay(500);
        R.g(-2500);

    }

}
