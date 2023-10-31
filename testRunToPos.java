package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="test run to pos", group="test")
public class testRunToPos extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!

        R.LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.LF.setTargetPosition(1500);
        R.LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R.setMtPower(0.3, 0.3, 0.3, 0.3);

    }

}
