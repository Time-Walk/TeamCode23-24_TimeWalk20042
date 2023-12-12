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
        Robot2023 R = new Robot2023(); // создание оъекта класса Robot2023
        R.initFields(telemetry, this, hardwareMap); // инициализация полей
        R.init(); // инициализация
        R.gamepad_init(gamepad1,gamepad2); // инициализация геймпада
        //R.KL.setPosition(R.servoPos);
//        while ( ! gamepad1.left_bumper && ! gamepad2.right_bumper ) { telemetry.addData("State", "Робот заблокирован"); telemetry.update(); }
//        telemetry.addData("ROBOT UNLOCKED!", "!");
//        telemetry.update();
        //telemetry = R.ftcDash.getTelemetry();
        //R.initNeonController.start();
        waitForStart(); // мтеод ожидания старта
        //R.NeState = "Driving";
        R.liftControllerT.start();  //Запуск работы лифта
        R.SM.setPosition(R.SMOPEN); // инициализация серво позиции серво мотора
        while (!isStopRequested()){ // пока нет запросов на остановку
            R.wheelbase();   //Передвижение колесной базы
            //R.servoController();    //Контроль серво на клешне
            //R.servoControllerPro();
            R.servoController(); // вызов управления сервами в телеопе
            R.driverHelper(); // вызов хэлпера, где ничего не написано :(
            //R.NeonController();
            //telemetry.addData("Close Pos (0.15 - default)", R.CLOSEPOS);
            //telemetry.addData("Default Pos (0.055 - default)", R.DEFPOS);
            //telemetry.update();
            telemetry.addData("KLCLOSE", R.KLCLOSE); // добавления информации в телеметрию
            telemetry.update(); // обновлние телеметрии
        }
        //R.LT.setPower(0);

    }
}
