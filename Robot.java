package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Robot {
    FtcDashboard ftcDash = FtcDashboard.getInstance();
    Telemetry telemetry; //Создование всех переменных
    //Telemetry telemetry = ftcDash.getTelemetry();
    Gamepad gamepad1;
    Gamepad gamepad2;
    LinearOpMode L;
    HardwareMap hwmp;

    void gamepad_init(Gamepad gamepad1, Gamepad gamepad2) { //Инициализация геймпадов
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

    }
    abstract void init(); //Инициализация
    abstract void initFields(Telemetry telemetry, LinearOpMode L, HardwareMap hwmp); //Инициализация

    public void delay(long millis) { //Вспомогательная функция как sleep, только прекраснее, чудеснее, чудеснее, прекраснее
        try {
            Thread.sleep(millis);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

}
