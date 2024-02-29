package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Тест", group="")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.addData("LF", 0);
        telemetry.addData("pwf", 0);
        telemetry.update();
        waitForStart();


        //okay, let's go!

        /*R.g(1700);
        R.delay(1000);
        R.g(-1700);
        R.delay(1000);
        R.g(1700);
        R.delay(1000);
        R.g(-1700);*/

        //R.rotate(90);
        //R.delay(1000);
        //R.rotate(-90);
        //R.delay(1000);
        //R.rotate(90);
        //R.delay(1000);
        //R.rotate(-90);

        R.XYaction(750, 0);

    }

}
