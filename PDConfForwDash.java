package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Новый уровень динозавриков", group="")
@Config
public class PDConfForwDash extends LinearOpMode {
    public static double kp = 0.00765;
    public static double kd = 0.003;
    public static double kpr = 0.08;
    public static double cc = 2000;
    public static double krelea = 0.15;
    public static double kpa = 0.00069;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.addData("LF", 0);
        telemetry.addData("Angle", 0);
        telemetry.update();
        waitForStart();

        //okay, let's go!
        if ( true ) {
            if ( true ) {
                telemetry = FtcDashboard.getInstance().getTelemetry(); // присваивание сути телеметрии из FTC Dashboard
                R.LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // обновление енкодер мотора
                R.LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // бег с использованием енкодера
                double ErLast = 0;
                //double RErLast = 0;
                //double Ir = 0;
                double Er0 = cc;
                double startRotation = R.getAngle();
                while ( this.opModeIsActive()) {
                    double pwf;

                    double Er = cc - R.LF.getCurrentPosition();

                    if ( Er > Er0 / 2 ) {
                        double Rele = krelea * Math.signum(Er);

                        double P = kpa * Math.abs(Er0 - Er);

                        pwf = Rele + P;

                        telemetry.addData("P", P);
                    }

                    else {
                        //kp = 0.27
                        //ki = 90
                        //kd = 0.0005625

                        //double kp = 0.3515;
                        //telemetry.addData("KP", kp);
                        double P = kp * Er;

                        //double ki = 0.0000537;
                        //Ir += Er;
                        //double I = Ir * ki;

                        //double kd = 0;
                        double ErD = ErLast - Er;
                        double D = kd * ErD;

                        if (Math.abs(D) > Math.abs(P)) {
                            D = P;
                        }

                        pwf = P + D;//I + D;


                        telemetry.addData("P", P);
                        telemetry.addData("D", D);

                    }

                    double REr = startRotation - R.getAngle();

                    //double Rkp = 0.04;
                    double RP = kpr * REr;

                    //double Rkd = 123;
                    //double RD = kdr * (RErLast - REr);

                    //if ( Math.abs(RD) > Math.abs(RP) ) { RD=RP; }

                    //RErLast = REr;

                    //double RLF = -RP;
                    //double RRB = -RP;

                    //telemetry.addData("RP", RP);
                    //telemetry.update();

                    ErLast = Er;

                    telemetry.addData("LF", R.LF.getCurrentPosition());
                    telemetry.addData("Angle", REr);
                    telemetry.addData("pwf", pwf);
                    telemetry.addData("RP", RP);
                    telemetry.addData("U", pwf+RP);
                    telemetry.addData("Er", Er);
                    //telemetry.addData("I", I);
                    //telemetry.addData("Ir", Ir);
                    telemetry.update();

                    R.LF.setPower(pwf-RP);
                    R.RB.setPower(-pwf-RP);
                }
                R.setMtZero();
            }

        }

    }

}
