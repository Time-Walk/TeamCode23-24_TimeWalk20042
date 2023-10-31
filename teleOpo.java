package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOp")
public class teleOpo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        R.gamepad_init(gamepad1,gamepad2);
        //R.KL.setPosition(R.servoPos);
//        while ( ! gamepad1.left_bumper && ! gamepad2.right_bumper ) { telemetry.addData("State", "Робот заблокирован"); telemetry.update(); }
//        telemetry.addData("ROBOT UNLOCKED!", "!");
//        telemetry.update();
        //telemetry = R.ftcDash.getTelemetry();
        //R.initNeonController.start();
        waitForStart();
        //R.NeState = "Driving";
        //R.liftControllerT.start();  //Запуск работы лифта
        while (!isStopRequested()){
            R.wheelbase();   //Передвижение колесной базы
            //R.servoController();    //Контроль серво на клешне
            //R.servoControllerPro();
            R.driverHelper();
            //R.NeonController();
            //telemetry.addData("Close Pos (0.15 - default)", R.CLOSEPOS);
            //telemetry.addData("Default Pos (0.055 - default)", R.DEFPOS);
            //telemetry.update();
        }
        //R.LT.setPower(0);

    }
}
