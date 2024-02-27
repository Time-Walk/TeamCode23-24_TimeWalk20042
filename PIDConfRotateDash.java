package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Динозаврики аааа", group="")
@Config
public class PIDConfRotateDash extends LinearOpMode {
    public static double kp = .5;
    public static double ki = .00015;
    public static double kd = .269;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.addData("pos", 0);
        telemetry.update();
        waitForStart();

        //okay, let's go!
        if ( true ) {
            if ( true ) {
                double degrees = 90;
                double pw = 1;
                double Er0 = -degrees;
                double errorFix = 0;
                double ErLast = 0;
                double Ir = 0;
                if (Er0 > 0) {
                    pw = -1;
                }
                degrees = R.getAngle() - degrees;
                if (degrees < -180) {
                    //degrees += 360;
                    Er0 = Er0 * -1;
                    pw *= -1;
                    errorFix = 1;
                }
                if (degrees > 180) {
                    //degrees -= 360;
                    Er0 = Er0 * -1;
                    pw *= -1;
                    errorFix = 2;
                }
                while ((Math.abs(degrees - R.getAngle()) > .8 || (Math.abs(degrees - R.getAngle()) - ErLast) > 1.2) && opModeIsActive() && !(gamepad1.right_stick_button)) {
                    if (R.getAngle() > 0 && errorFix == 1) {
                        Er0 = Er0 * -1;
                        degrees += 360;
                        pw *= -1;
                        errorFix = 0;
                    }
                    if (R.getAngle() < 0 && errorFix == 2) {
                        Er0 = Er0 * -1;
                        degrees -= 360;
                        pw *= -1;
                        errorFix = 0;
                    }

                    double Er = degrees - (R.getAngle());

                    //double kp = 0.0012;
                    double P = kp * Er / Er0 * pw;

                    Ir += -Er * ki;
                    //double ki = 1.2;
                    double I = Ir;

                    //double kd = 0.00003;
                    double ErD = Er - ErLast;
                    double D = kd * ErD * (1 / Er);

                    if (Math.abs(D) > Math.abs(P)) {
                        D = P;
                    }

                    double kr = -0.075;
                    double Rele = kr * Math.signum(Er);

                    ErLast = Er;


                    double pwf = P + I + D; //Регулятор


                    R.setMtPower(pwf, pwf, pwf, pwf);

                    telemetry.addData("pos", R.getAngle());
                    telemetry.addData("pwf", pwf);
                    telemetry.addData("kp", kp);
                    telemetry.addData("P", P);
                    telemetry.addData("kd", kd);
                    telemetry.addData("D", D);
                    telemetry.addData("ki", ki);
                    telemetry.addData("I", I);
                    telemetry.addData("dEr", ErD);
                    telemetry.update();
                }
                R.setMtZero();
            }
        }

    }

}
