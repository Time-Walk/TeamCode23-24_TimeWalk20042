package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Именно поэтому свеча и совершает свои колебания", group="")
public class CiglerNikolsMethod extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023();
        R.initFields(telemetry, this, hardwareMap);
        R.init();

        FtcDashboard ftcDash = FtcDashboard.getInstance();
        Telemetry telemetry = ftcDash.getTelemetry();

        waitForStart();

        //okay, let's go!

        R.LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double kp = 0;

        while (true) {

                if (R.gamepad1.dpad_up  ) { kp += 0.00001; }
                if (R.gamepad1.dpad_down) { kp -= 0.00001; }

                    double Er = -R.LF.getCurrentPosition();

                    double P = kp * Er ;

                    double pwf = P;

                    R.LF.setPower(pwf);
                    R.RB.setPower(-pwf);

                    //R.setMtPower(pwf, pwf, pwf, pwf);

                    telemetry.addData("kp", kp);
                    telemetry.addData("Pos", R.LF.getCurrentPosition());
                    telemetry.update();

                    R.delay(10);
            }
        }

    }

