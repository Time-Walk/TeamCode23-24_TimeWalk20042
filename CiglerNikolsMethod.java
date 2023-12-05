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

        double kp = 0.001;

        boolean dup = false;
        boolean ddown = false;

        while (true) {

                if (gamepad1.dpad_up  ) {
                    if ( ! dup ) {
                        kp += 0.001;
                        dup = true;
                    }
                }
                else {
                    dup = false;
                }
                if (gamepad1.dpad_down) {
                    if ( ! ddown ) {
                        kp -= 0.001;
                        ddown = true;
                    }
                }
                else {
                    ddown = false;
                }
                    double Er = -R.getAngle();

                    double P = kp * Er ;

                    double pwf = P;

                    //R.LF.setPower(pwf);
                    //R.RB.setPower(-pwf);

                    R.setMtPower(pwf, pwf, pwf, pwf);

                    telemetry.addData("Er", Er);
                    telemetry.addData("P", P);
                    telemetry.addData("pwf", pwf);
                    telemetry.addData("kp", kp);
                    telemetry.addData("Pos", R.LF.getCurrentPosition());
                    telemetry.update();

                    R.delay(10);
            }
        }

    }

//kp = 0.27
//ki = 54
//kd = 0.0003375

//kp = 0.0012
//ki = 1.2
//kd = 0.00003

