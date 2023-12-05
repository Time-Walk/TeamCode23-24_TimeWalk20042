package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Динозаврики", group="")
public class PIDConfRotate extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        telemetry = FtcDashboard.getInstance().getTelemetry();
        waitForStart();

        //okay, let's go!

        double step = 0;
        double kp = 0;
        double ki = 0;
        double kd = 0;
        double kpmem = 0.3515;
        double kimem = 0.0000537;
        double kdmem = 0;
        String change = "kp";
        boolean isRSB = false;
        boolean isLSB = false;
        while ( true ) {
            //R.wheelbase();
            if ( gamepad1.right_stick_button ) {
                if ( !isRSB ) {
                    switch (change) {
                        case "kp":
                            change = "ki";
                            break;
                        case "kd":
                            change = "kp";
                            break;
                        case "ki":
                            change = "kd";
                            break;
                    }
                    isRSB = true;
                }
            }
            else { isRSB = false; }
            if ( gamepad1.left_stick_button ) {
                if ( !isLSB ) {
                    switch (change) {
                        case "kp":
                            change = "kd";
                            break;
                        case "kd":
                            change = "ki";
                            break;
                        case "ki":
                            change = "kp";
                            break;
                    }
                    isLSB = true;
                }
            }
            else { isLSB = false; }
            if ( gamepad1.x ) {
                step = 0.0000000001;
            }
            if ( gamepad1.b ) {
                step = 0.000001;
            }
            if ( gamepad1.a ) {
                step = 0.001;
            }
            if ( gamepad1.y ) {
                step = 0.000000000001;
            }
            if ( gamepad1.dpad_up ) {
                switch ( change ) {
                    case "kp":
                        kp += step;
                        break;
                    case "kd":
                        kd += step;
                        break;
                    case "ki":
                        ki += step;
                        break;
                }
            }
            if ( gamepad1.dpad_down ) {
                switch ( change ) {
                    case "kp":
                        kp -= step;
                        break;
                    case "kd":
                        kd -= step;
                        break;
                    case "ki":
                        ki -= step;
                        break;
                }
            }
            if ( gamepad1.dpad_left ) {
                switch ( change ) {
                    case "kp":
                        kp = 0;
                        break;
                    case "kd":
                        kd = 0;
                        break;
                    case "ki":
                        ki = 0;
                        break;
                }
            }
            if ( gamepad1.dpad_right) {
                switch ( change ) {
                    case "kp":
                        kp = kpmem;
                        break;
                    case "kd":
                        kd = kdmem;
                        break;
                    case "ki":
                        ki = kimem;
                        break;
                }
            }
            if ( gamepad1.left_trigger > 0.9 ) {
                switch ( change ) {
                    case "kp":
                        kpmem = kp;
                        break;
                    case "kd":
                        kdmem = kd;
                        break;
                    case "ki":
                        kimem = ki;
                        break;
                }
            }
            if ( gamepad1.right_bumper && gamepad1.left_bumper ) {
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
                while (Math.abs(degrees - R.getAngle()) > 3 && opModeIsActive() && !(gamepad1.right_stick_button)) {
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

                    Ir += -Er;
                    //double ki = 1.2;
                    double I = Ir * ki;

                    //double kd = 0.00003;
                    double ErD = Er - ErLast;
                    double D = kd * ErD * (1 / Er);

                    if (Math.signum(D) > Math.signum(P)) {
                        D = P;
                    }

                    double kr = -0.075;
                    double Rele = kr * Math.signum(Er);

                    ErLast = Er;


                    double pwf = P + I + D; //Регулятор


                    R.setMtPower(pwf, pwf, pwf, pwf);

                    telemetry.addData("pos", R.getAngle());
                    telemetry.addData("pwf", pwf);
                    telemetry.addData("step", step);
                    telemetry.addData("kp", kp);
                    telemetry.addData("P", P);
                    telemetry.addData("kd", kd);
                    telemetry.addData("D", D);
                    telemetry.addData("ki", ki);
                    telemetry.addData("I", I);
                    telemetry.addData("change", change);
                    telemetry.addData("kpmem", kpmem);
                    telemetry.addData("kimem", kimem);
                    telemetry.addData("kdmem", kdmem);
                    telemetry.update();
                }
                R.setMtZero();
                isRSB = true;
            }
            telemetry.addData("pos", R.getAngle());
            telemetry.addData("pwf", 0);
            telemetry.addData("step", step);
            telemetry.addData("kp", kp);
            telemetry.addData("P", 0);
            telemetry.addData("kd", kd);
            telemetry.addData("D", 0);
            telemetry.addData("ki", ki);
            telemetry.addData("I", 0);
            telemetry.addData("change", change);
            telemetry.addData("kpmem", kpmem);
            telemetry.addData("kimem", kimem);
            telemetry.addData("kdmem", kdmem);
            telemetry.update();
        }

    }

}
