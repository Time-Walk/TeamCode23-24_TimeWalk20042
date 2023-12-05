package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="СН.ПР.Первое Простое", group="blue")
public class Blue_Left_First extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!

        R.LT.setPower(0.4);
        R.delay(500);
        R.LT.setPower(R.HOLDPOWER);
        R.setMtPower(0.35, 0.35, -0.35, -0.35);
        R.delay(400);
        R.setMtZero();
        R.setMtPower(-0.6, 0.6, -0.6, 0.6);
        R.delay(2500);
        R.setMtZero();
        //R.delay(100);
        //R.rotate(-45);
        //R.g(-300);

    }

}
