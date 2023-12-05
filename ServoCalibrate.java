package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="калибровка серво", group="")
public class ServoCalibrate extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!

        double pos = 0.5;

        boolean dup = false;
        boolean ddown = false;

        while ( true ) {
            if (gamepad1.x) {
                pos = 0;
            }
            else if (gamepad1.a) {
                pos = 0.5;
            }
            else if (gamepad1.b) {
                pos = 1;
            }
            else {
                if (gamepad1.dpad_up) {
                    if ( !dup ) {
                        pos += 0.05;
                        dup = true;
                    }
                }
                else {
                    dup = false;
                }
                if (gamepad1.dpad_down) {
                    if ( !ddown ) {
                        pos -= 0.05;
                        ddown = true;
                    }
                }
                else {
                    ddown = false;
                }
            }
            R.SM.setPosition(pos);
            telemetry.addData("pos", pos);
            telemetry.update();
        }

    }

}
