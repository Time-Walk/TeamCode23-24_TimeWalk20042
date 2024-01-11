package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Динозаврики аааа go", group="")
@Config
public class PIDConfForwDash extends LinearOpMode {
    public static double kp = 0.0006;
    public static double ki = 0.0000008;
    public static double kd = 0.003;
    public static double kpr = 0.1;
    public static double kdr = 0.001;
    public static double cc = 2000;
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
                double RErLast = 0;
                double Ir = 0;
                double startRotation = R.getAngle();
                while ( this.opModeIsActive()) {

                    double Er = cc - R.LF.getCurrentPosition();

                    //kp = 0.27
                    //ki = 90
                    //kd = 0.0005625

                    //double kp = 0.3515;
                    //telemetry.addData("KP", kp);
                    double P = kp * Er ;

                    //double ki = 0.0000537;
                    Ir += Er;
                    double I = Ir * ki;

                    //double kd = 0;
                    double ErD = ErLast - Er;
                    double D = kd * ErD;

                    if ( Math.abs(D) > Math.abs(P) ) { D=P; }

                    double REr = startRotation - R.getAngle();

                    //double Rkp = 0.04;
                    double RP = kpr * REr;

                    //double Rkd = 123;
                    double RD = kdr * (RErLast - REr);

                    if ( Math.abs(RD) > Math.abs(RP) ) { RD=RP; }

                    RErLast = REr;

                    //double RLF = -RP;
                    //double RRB = -RP;

                    //telemetry.addData("RP", RP);
                    //telemetry.update();

                    ErLast = Er;

                    double pwf = P + I + D;

                    telemetry.addData("LF", R.LF.getCurrentPosition());
                    telemetry.addData("Angle", REr);
                    telemetry.addData("pwf", pwf);
                    telemetry.addData("RP", RP);
                    telemetry.addData("U", pwf+RP);
                    telemetry.addData("Er", Er);
                    telemetry.addData("P", P);
                    telemetry.addData("I", I);
                    telemetry.addData("Ir", Ir);
                    telemetry.addData("D", D);
                    telemetry.update();

                    R.LF.setPower(pwf-RP);
                    R.RB.setPower(-pwf-RP);
                }
                R.setMtZero();
            }

        }

    }

}
