package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
//import com.vuforia.CameraDevice;
//import com.vuforia.Image;
//import com.vuforia.PIXEL_FORMAT;
//import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class Robot2023 extends Robot {
    OpenCvCamera camera; // инициализация переменной камеры
    DcMotor RF, LF, RB, LB, LT, NE; // инициализация моторов постоянного тока
    // Servo KL;//, KL1;
    Servo SM; // инициализация переменной для запуска самолётика
    BNO055IMU imu; //Акселерометр

    //VuforiaLocalizerImpl vuforia;

    Orientation angles; // переменная в которой будет храниться угол поворота под акселерометр
    Acceleration gravity; // здесь храниться важная информация для акселерометра

    VoltageSensor vs; // инициализация переменной датчика напряжения

    @Override
    void init() { //Инициализация:

        LF = hwmp.get(DcMotor.class, "LF");
        LB = hwmp.get(DcMotor.class, "LB");
        RB = hwmp.get(DcMotor.class, "RB");
        RF = hwmp.get(DcMotor.class, "RF");
        //  присваивание экземпляра к конфигу, колёсная база

        LT = hwmp.get(DcMotor.class, "LT");
        // лифт

        //NE = hwmp.get(DcMotor.class, "NE");

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Режим остоновки: торможение
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // KL = hwmp.get(Servo.class, "KL");
        //KL1 = hwmp.get(Servo.class, "KL1");

        SM = hwmp.get(Servo.class, "SM"); // инициализация серво мотора на механизме запуска самолётика


        vs = hwmp.voltageSensor.iterator().next(); // присваивание экземпляра датчика

        //initVuforia();

        initCamera(); // вызов функции инициализации камеры

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); // инициализация Акселерометра
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwmp.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()) { //Калибровка акселерометра
            delay(30);
            telemetry.addData("Wait", "Calibration"); //Сообщение о калибровке
            telemetry.update();
        }
        telemetry.addData("Done!", "Calibrated"); //Сообщение об окончании калибровки
        telemetry.update();

        //NeState = "Hold";
    }

    @Override
    void initFields(Telemetry telemetry, LinearOpMode L, HardwareMap hwmp) { //Инициализация
        this.telemetry = telemetry;
        this.L = L;
        this.hwmp = hwmp;


    }

    public void initCamera() { // инициализация камеры
        int cameraMonitorViewId = hwmp.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwmp.appContext.getPackageName());
        // получение Id монитора камеры
        WebcamName webcamName = hwmp.get(WebcamName.class, "Webcam"); // получение имени камеры из конфига
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId); // получение экземпляра камеры

    }

    /*public void initVuforia() {
        final String VUFORIA_KEY = "AXmpKJj/////AAABmcT291bCTEQeoCRi1kDOyP8ApoUammAer00aO1owWHeTV7AmOtKkjy/8jRV99nQLFDMqq8eFrFk02tC3e24Hk9u4pnB+m2zRTTtVlIJ9G248PtXINEGUoPi+W2t53dbLT5+RSxBdMGDAKC7aeTv0tlpN1zNLnxYbVKqgbsIKU5C5FOGByrJU7xGP/qLDsY/TAlNbcq73vL9ClSAGo0Im+77mABDEXUVZilP05IR5sbXJYHo/J9O2U8ZfX4KnpnNbPWzzGBFpyKrVRNYihX7s/pjlitY6Fok2sQ+PX4XDoCu3dw/9rtnqpMwTkBtrzvmVuL01zVmKcf8e31FWafJ2I1CBJ5t2OJbrOO0m4NiELoUL";

        OpenGLMatrix targetPose     = null;
        String targetName           = "";

        int cameraMonitorViewId = hwmp.appContext.getResources().getIdentifier("Camera", "id", hwmp.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersWebcam = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parametersWebcam.vuforiaLicenseKey = VUFORIA_KEY;
        parametersWebcam.useExtendedTracking = false;
        parametersWebcam.cameraName = hwmp.get(WebcamName.class, "Webcam");
        parametersWebcam.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = new VuforiaLocalizerImpl(parametersWebcam);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);
        CameraDevice.getInstance().setFlashTorchMode(true);


    }*/

    /*Bitmap getImage() throws InterruptedException {
        Image img;
        img = getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565);
        Bitmap btm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
        btm.copyPixelsFromBuffer(img.getPixels());
        return btm;

    }*/

    /*Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int format) {
        long NI = frame.getNumImages();
        for (int i = 0; i < NI; i++) {
            if (frame.getImage(i).getFormat() == format) {
                return frame.getImage(i);
            }
        }
        return null;*/
    //}

    /*boolean ConusRq(double rr, double gr, double br, double angle, double r, double g, double b) {
        double anglerr = Math.acos((rr * r + gr * g + br * b) / (Math.sqrt(rr * rr + gr * gr + br * br) * Math.sqrt(r * r + g * g + b * b)));
        //telemetry.addData("anglerr", anglerr);
        //telemetry.addData("anglenon", angle);
        if (angle > anglerr) {
            return true;
        }
        return false;
    }*/

    /*void Katet(double cm, int direction) { //direction: 1 - forward, 2 - right, 3 - backward, 4 - left
        double lbs = 1; // lb signum
        double rbs = 1;
        double lfs = 1;
        double rfs = 1;
        double kc = 1;
        if (direction == 1) {
            rbs = -1;
            rfs = -1;
        }
        if (direction == 2) {
            lbs = -1;
            rbs = -1;
        }
        if (direction == 3) {
            lbs = -1;
            lfs = -1;
            //kc = 2;
        }
        if (direction == 4) {
            lfs = -1;
            rfs = -1;
            //kc = 2;
        }
        double pw = 1;
        double ccx = ((400 * cm) / 32.97)*kc;
        double cc = ccx/Math.sqrt(2);
        double Er0 = cc;
        double ErLast = 0;
        double D = 1;
        double startAngle = getAngle();
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while ( ( Math.abs(D) > 0.1 ||  Math.abs(cc) - Math.abs(LF.getCurrentPosition()) > 10*Math.signum(cc)) && L.opModeIsActive()) {

            double Er = Math.abs(cc) - Math.abs(LF.getCurrentPosition());

            double kp = 0.8;
            double P = kp * Er / Er0 * pw ;

            double kd = 0.2;
            double ErD = Er - ErLast;
            D = kd * ErD * (1 / Er);

            if (Math.signum(D) > Math.signum(P)) {  D=P; }

            double kr = 0.2*Math.signum(cc);
            double Rele = kr * Math.signum(Er);

            ErLast = Er;

            double lbr = 0; // lb rotate
            double lfr = 0;
            double rfr = 0;
            double rbr = 0;
            double rotP =0;

            if ( Math.abs(startAngle - getAngle()) > 1 ) {
                double rotkp = 0.026;
                rotP = -(rotkp * (startAngle - getAngle()));
                if ( direction == 1 ) {
                    rfr += rotP;
                    rbr += rotP;
                }
                if ( direction == 2 ) {
                    lbr += rotP;
                    rbr += rotP;
                }
                if ( direction == 3 ) {
                    lfr += rotP;
                    lbr += rotP;
                }
                if ( direction == 4 ) {
                    rfr += rotP;
                    lfr += rotP;
                }
            }


            double pwf = (pw * (P+D+Rele))*Math.signum(Er); //Регулятор

            if ( rotP > pwf ) { rotP = pwf*0.6; }

            //telemetry.addData("currPosition", LF.getCurrentPosition());
            //telemetry.addData("cc", cc);
            //telemetry.addData("Err", Er);
            //telemetry.addData("cond1", cc - LF.getCurrentPosition());
            //telemetry.addData("cond2", 10*Math.signum(cc));
            //telemetry.addData("pwf", pwf);
            //telemetry.addData("D", D);
            //telemetry.update();

            setMtPower(pwf*lfs+lfr, pwf*lbs+lbr, pwf*rfs+rfr, pwf*rbs+rbr);


            /*telemetry.addData("cc", cc);
            telemetry.addData("Er0", Er0);
            telemetry.addData("Er", Er);
            telemetry.addData("getCurrentPosition", LB.getCurrentPosition());
            telemetry.addData("kp", kp);
            telemetry.addData("Rele", Rele);
            telemetry.addData("D", D);
            telemetry.addData("pw", pw);
            telemetry.addData("pwf", pwf);
            telemetry.addData("rotEr", startAngle-getAngle());
            telemetry.addData("lfr", lfr);
            telemetry.addData("lbr", lbr);
            telemetry.addData("rfr", rfr);
            telemetry.addData("rbr", rbr);
            telemetry.addData("rotP", rotP);
            telemetry.update();*/

    //}

        /*delay(500);

        while ( Math.abs(startAngle - getAngle()) > 1  && L.opModeIsActive()) {

            double kp = 0.02;
            double P = -(kp * (startAngle - getAngle()));

            setMtPower(P, P, P, P);

        }

        setMtPower(0, 0, 0, 0);

        delay(500);
    }

    void katetPlus(double cm, int direction, double kp, double kd, boolean isRot) { //direction: 1 - forward, 2 - right, 3 - backward, 4 - left
        double lbs = 1; // lb signum
        double rbs = 1;
        double lfs = 1;
        double rfs = 1;
        double kc = 1;
        if (direction == 1) {
            rbs = -1;
            rfs = -1;
        }
        if (direction == 2) {
            lbs = -1;
            rbs = -1;
        }
        if (direction == 3) {
            lbs = -1;
            lfs = -1;
            //kc = 2;
        }
        if (direction == 4) {
            lfs = -1;
            rfs = -1;
            //kc = 2;
        }
        double pw = 1;
        double ccx = ((400 * cm) / 32.97)*kc;
        double cc = ccx/Math.sqrt(2);
        double Er0 = cc;
        double ErLast = 0;
        double D = 1;
        double startAngle = getAngle();
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while ( ( Math.abs(D) > 0.1 ||  Math.abs(cc) - Math.abs(LF.getCurrentPosition()) > 10*Math.signum(cc)) && L.opModeIsActive()) {

            double Er = Math.abs(cc) - Math.abs(LF.getCurrentPosition());

            double P = kp * Er / Er0 * pw ;

            double ErD = Er - ErLast;
            D = kd * ErD * (1 / Er);

            if (Math.signum(D) > Math.signum(P)) {  D=P; }

            double kr = 0.2*Math.signum(cc);
            double Rele = kr * Math.signum(Er);

            ErLast = Er;

            double lbr = 0; // lb rotate
            double lfr = 0;
            double rfr = 0;
            double rbr = 0;
            double rotP =0;

            if ( Math.abs(startAngle - getAngle()) > 1  && isRot ) {
                double rotkp = 0.026;
                rotP = -(rotkp * (startAngle - getAngle()));
                if ( direction == 1 ) {
                    rfr += rotP;
                    rbr += rotP;
                }
                if ( direction == 2 ) {
                    lbr += rotP;
                    rbr += rotP;
                }
                if ( direction == 3 ) {
                    lfr += rotP;
                    lbr += rotP;
                }
                if ( direction == 4 ) {
                    rfr += rotP;
                    lfr += rotP;
                }
            }


            double pwf = (pw * (P+D+Rele))*Math.signum(Er); //Регулятор

            if ( rotP > pwf ) { rotP = pwf*0.6; }

            //telemetry.addData("currPosition", LF.getCurrentPosition());
            //telemetry.addData("cc", cc);
            //telemetry.addData("Err", Er);
            //telemetry.addData("cond1", cc - LF.getCurrentPosition());
            //telemetry.addData("cond2", 10*Math.signum(cc));
            //telemetry.addData("pwf", pwf);
            //telemetry.addData("D", D);
            //telemetry.update();

            setMtPower(pwf*lfs+lfr, pwf*lbs+lbr, pwf*rfs+rfr, pwf*rbs+rbr);


            /*telemetry.addData("cc", cc);
            telemetry.addData("Er0", Er0);
            telemetry.addData("Er", Er);
            telemetry.addData("getCurrentPosition", LB.getCurrentPosition());
            telemetry.addData("kp", kp);
            telemetry.addData("Rele", Rele);
            telemetry.addData("D", D);
            telemetry.addData("pw", pw);
            telemetry.addData("pwf", pwf);
            telemetry.addData("rotEr", startAngle-getAngle());
            telemetry.addData("lfr", lfr);
            telemetry.addData("lbr", lbr);
            telemetry.addData("rfr", rfr);
            telemetry.addData("rbr", rbr);
            telemetry.addData("rotP", rotP);
            telemetry.update();*/

    //}

        /*delay(500);

        while ( (Math.abs(startAngle - getAngle()) > 1  && L.opModeIsActive()) && isRot ) {

            kp = 0.02;
            double P = -(kp * (startAngle - getAngle()));

            setMtPower(P, P, P, P);

        }

        setMtPower(0, 0, 0, 0);

        delay(500);
    }

    void go(double cm) { //
        double pw = 1;
        double cc = (400 * cm) / 32.97;
        double Er0 = cc;
        double errorFix=0;
        double ErLast = 0;
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*if (degrees < -180) {
            degrees += 360;
            pw = pw * -1;
        }
        if (degrees > 180) {
            degrees -= 360;
            pw = pw * -1;
        }*/
        /*double D = 1;
        //while (LB.getCurrentPosition() < m) { setMtPower(-pw, -pw, pw, pw); }
        while ( ( Math.abs(D) > 0.1 ||  Math.abs(cc) - Math.abs(LF.getCurrentPosition()) > 10*Math.signum(cc)) && L.opModeIsActive()) {

            double Er = Math.abs(cc) - Math.abs(LF.getCurrentPosition());

            double kp = 0.9;
            double P = kp * Er / Er0 * pw;

            double kd = 0.15;
            double ErD = Er - ErLast;
            D = kd * ErD * (1 / Er);

            if (Math.signum(D) > Math.signum(P)) {  D=P; }

            double kr = 0.2*Math.signum(cc);
            double Rele = kr * Math.signum(Er);

            ErLast = Er;



            double pwf = (pw * (P+D+Rele))*Math.signum(Er); //Регулятор

            //telemetry.addData("currPosition", LF.getCurrentPosition());
            //telemetry.addData("cc", cc);
            //telemetry.addData("Err", Er);
            //telemetry.addData("cond1", cc - LF.getCurrentPosition());
            //telemetry.addData("cond2", 10*Math.signum(cc));
            //telemetry.addData("pwf", pwf);
            //telemetry.addData("D", D);
            //telemetry.update();

            LF.setPower(pwf);
            RB.setPower(-pwf);


            /*telemetry.addData("cc", cc);
            telemetry.addData("Er0", Er0);
            telemetry.addData("Er", Er);
            telemetry.addData("getCurrentPosition", LB.getCurrentPosition());
            telemetry.addData("kp", kp);
            telemetry.addData("Rele", Rele);
            telemetry.addData("D", D);
            telemetry.addData("pw", pw);
            telemetry.addData("pwf", pwf);
            telemetry.update();*/

    //}

        /*LF.setPower(0);
        RB.setPower(0);

        delay(500);
    }

    void g(double cc) {
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double ErLast = 0;
        double Ir = 0;
        double startRotation = getAngle();
        while ( Math.abs(cc - LF.getCurrentPosition()) > 5 && L.opModeIsActive()) {

            double Er = cc - LF.getCurrentPosition();

            double kp = 0.0005;
            telemetry.addData("KP", kp);
            double P = kp * Er ;

            double ki = 0.000002;
            Ir += Er;
            double I = Ir * ki;

            double kd = 0.00008;
            double ErD = ErLast - Er;
            double D = kd * ErD;

            if ( Math.abs(D) > Math.abs(P) ) { D=P; }

            double REr = startRotation - getAngle();

            double Rkp = 0.04;
            double RP = Rkp * REr;

            double RLF = -RP;
            double RRB = -RP;

            telemetry.addData("RP", RP);
            telemetry.update();

            double pwf = P + I;

            LF.setPower(pwf+RLF);
            RB.setPower(-pwf+RRB);
        }
        setMtZero();
    }*/

    double getAngle() { //Функция получения данных с акселерометра
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        return angles.firstAngle;
    }

    /*void rotate (double degrees) { //Функция автонома: поворот
        double pw = 1;
        double Er0 = -degrees;
        double errorFix=0;
        double ErLast = 0;
        double Ir = 0;
        if (Er0 > 0) { pw = -1; }
        degrees = getAngle() - degrees;
        if (degrees < -180) {
            //degrees += 360;
            Er0 = Er0 * -1;
            pw *= -1;
            errorFix=1;
        }
        if (degrees > 180) {
            //degrees -= 360;
            Er0 = Er0 * -1;
            pw *= -1;
            errorFix=2;
        }
        while ( Math.abs(degrees - getAngle()) > 3  && L.opModeIsActive()) {
            if (getAngle() > 0 && errorFix==1) { Er0 = Er0 * -1; degrees += 360; pw *= -1; errorFix=0; }
            if (getAngle() < 0 && errorFix==2) { Er0 = Er0 * -1; degrees -= 360; pw *= -1; errorFix=0; }

            double Er = degrees - (getAngle());

            double kp = 0.1;
            double P = kp * Er / Er0 * pw;

            Ir += -Er;
            double ki = 0.0002;
            double I = Ir * ki;

            double kd = 0.1;
            double ErD = Er - ErLast;
            double D = kd * ErD * (1/Er);

            if (Math.signum(D) > Math.signum(P)) {  D=P; }

            double kr = -0.075;
            double Rele = kr * Math.signum(Er);

            ErLast = Er;


            double pwf = P + I; //Регулятор


            LB.setPower(pwf);
            LF.setPower(pwf);
            RB.setPower(pwf);
            RF.setPower(pwf);

                /*telemetry.addData("degrees", degrees);
                telemetry.addData("getAngle()", getAngle());
                telemetry.addData("Er (degrees - getAngle)", Er);
                telemetry.addData("Er0", Er0);
                telemetry.addData("kp", kp);
                telemetry.addData("P", P);
                telemetry.addData("D", D);
                telemetry.addData("Rele", Rele);
                telemetry.addData("pw", pw);
                telemetry.addData("pwf", pwf);
                telemetry.update();*/

    //}
        /*LB.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        RF.setPower(0);
        delay(500);
    }

    void rotateS(double degrees) { //Функция автонома: поворот
        double pw = 1;
        double Er0 = -degrees;
        double errorFix=0;
        double ErLast = 0;
        if (Er0 > 0) { pw = -1; }
        degrees = getAngle() - degrees;
        if (degrees < -180) {
            //degrees += 360;
            Er0 = Er0 * -1;
            pw *= -1;
            errorFix=1;
        }
        if (degrees > 180) {
            //degrees -= 360;
            Er0 = Er0 * -1;
            pw *= -1;
            errorFix=2;
        }
        while ( Math.abs(degrees - getAngle()) > 3  && L.opModeIsActive()) {
            if (getAngle() > 0 && errorFix==1) { Er0 = Er0 * -1; degrees += 360; pw *= -1; errorFix=0; }
            if (getAngle() < 0 && errorFix==2) { Er0 = Er0 * -1; degrees -= 360; pw *= -1; errorFix=0; }

            double Er = degrees - (getAngle());

            double kp = 0.48;
            double P = kp * Er / Er0 * pw;

            double kd = 0.1;
            double ErD = Er - ErLast;
            double D = kd * ErD * (1/Er);

            if (Math.signum(D) > Math.signum(P)) {  D=P; }

            double kr = -0.025;
            double Rele = kr * Math.signum(Er);

            ErLast = Er;


            double pwf = P; //Регулятор


            LB.setPower(pwf);
            LF.setPower(pwf);
            RB.setPower(pwf);
            RF.setPower(pwf);

                /*telemetry.addData("degrees", degrees);
                telemetry.addData("getAngle()", getAngle());
                telemetry.addData("Er (degrees - getAngle)", Er);
                telemetry.addData("Er0", Er0);
                telemetry.addData("kp", kp);
                telemetry.addData("P", P);
                telemetry.addData("D", D);
                telemetry.addData("Rele", Rele);
                telemetry.addData("pw", pw);
                telemetry.addData("pwf", pwf);
                telemetry.update();*/

    //}
        /*LB.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        RF.setPower(0);
        delay(500);
    }*/

    void DEBUG() {

    }

    /*void setLift(double ticks) {
        LT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double ErLast = 0;
        double rI = 0;
        //double dt = 0;
        while ( Math.abs(ticks - LT.getCurrentPosition()) > 5 && L.opModeIsActive() ) {
            //dt += 1;
            double Er = ticks - LT.getCurrentPosition();

            double kp = 0.0013;
            double P = kp * Er;

            double ki = 0.00003;
            rI = rI + Er;
            double I = rI * ki;

            double kd = 0.0002;
            double ErD = Er - ErLast;
            double D = kd * ErD;

            double pwf = P + I + D;

            ErLast = Er;

            telemetry.addData("LT", LT.getCurrentPosition());
            telemetry.addData("Err", Er);
            telemetry.addData("P", P);
            telemetry.addData("rI", rI);
            telemetry.addData("I", I);
            telemetry.addData("ErD", ErD);
            telemetry.addData("D", D);
            telemetry.addData("pwf", pwf);
            telemetry.update();

            LT.setPower(pwf);

            delay(50);
        }
        telemetry.addData("Work ended in", LT.getCurrentPosition());
        LT.setPower(0);
        delay(50);
    }*/

    void wheelbase() { // метод про колёсную базу
        double rf = gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x * 0.6) - (gamepad1.left_trigger * 0.6) + (gamepad1.right_trigger * 0.6); // получение для 4 колёс необходимой мощности
        double rb = gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x * 0.6) - (gamepad1.left_trigger * 0.6) + (gamepad1.right_trigger * 0.6);
        double lf = -gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x * 0.6) - (gamepad1.left_trigger * 0.6) + (gamepad1.right_trigger * 0.6);
        double lb = -gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x * 0.6) - (gamepad1.left_trigger * 0.6) + (gamepad1.right_trigger * 0.6);
        RF.setPower(rf); // задаём мощности
        RB.setPower(rb);
        LF.setPower(lf);
        LB.setPower(lb);
    }

    /*Thread initNeonController = new Thread() {
        @Override
        public void run() {
            while (!L.opModeIsActive() && !L.isStopRequested()) {
                NeonController();
            }
        }
    };*/

    /*Thread AutoNeonController = new Thread() {
        @Override
        public void run() {
            while (L.opModeIsActive() && !L.isStopRequested()) {
                NeonController();
            }
        }
    };*/

    /*double HOLDPOWER = 0.0008;
    Thread liftControllerT = new Thread() { //Поток для лифта
        @Override
        public void run() {
            boolean hold = false;
            double Power = 0;
            while (L.opModeIsActive() && !L.isStopRequested()) {
                //telemetry.addData("y", gamepad2.left_stick_y);
                //telemetry.update();
                LT.setPower((gamepad2.right_stick_y/-2.6769)+Power); //Управление лифтом стиком
                if (gamepad2.a) {
                    if ( hold ) {
                        Power = 0;
                        hold = false;
                        delay(300);
                    }
                    else {
                        Power = HOLDPOWER;
                        hold = true;
                        delay(300);
                    }
                }
                if (gamepad2.x) {
                    NeState = "Plink";
                    plinkCount = 1;
                    setLift(240);
                    Power = HOLDPOWER;
                    hold = true;
                }
                if (gamepad2.b) {
                    NeState = "Plink";
                    plinkCount = 2;
                    setLift(390);
                    Power = HOLDPOWER;
                    hold = true;
                }
                if (gamepad2.y) {
                    NeState = "Plink";
                    plinkCount = 3;
                    setLift(545);
                    Power = HOLDPOWER;
                    hold = true;
                }
            }
        }
    };*/


    void setMtPower(double lf, double lb, double rf, double rb) { // метод про получение мощности
        LF.setPower(lf); // присваивание мощности
        LB.setPower(lb);
        RF.setPower(rf);
        RB.setPower(rb);
    }

    void setMtZero() {
        setMtPower(0, 0, 0, 0);
    } // остановка мотора

    /*int MgI = 0;
    int GrI = 0;
    int CnI = 0;
    void analyze(int red, int green, int blue) {
        boolean Mg = ConusRq(220, 70, 120, 0.2, red, green, blue);
        boolean Gr = ConusRq(50, 120, 50, 0.3, red, green, blue);
        boolean Cn = ConusRq(50, 130, 140, 0.15, red, green, blue);
        if (Mg) {MgI=MgI+1; telemetry.addData("Mg detecked! Total count", MgI); }
        if (Gr) {GrI=GrI+1; telemetry.addData("Gr detecked! Total count", GrI); }
        if (Cn) {CnI=CnI+1; telemetry.addData("Cn detecked! Total count", CnI); }
    }*/

    void driverHelper() {
        //if ( gamepad1.x ) { rotate(90); }
        //if ( gamepad1.b ) { rotate(90); }
    }

    //double servoPos=0.1;
    /*double CLOSEPOS = 0.9;
    double OPENPOS = 0.8;
    boolean isDpadL = false;
    boolean isDpadR = false;
    boolean leftS = false;
    boolean rightS = false;
    void servoController() {
        if (gamepad2.left_bumper ) {
            //if ( !leftS ) {
            KL.setPosition(OPENPOS); leftS = true; }//} else { leftS = false; }
        if (gamepad2.right_bumper) {
            //if ( !rightS ) {
            KL.setPosition(CLOSEPOS); rightS = true; }//} else { rightS = false; }
        if (gamepad2.dpad_left) {
            if ( !isDpadL ) {
                isDpadL = true;
                CLOSEPOS -= 0.01;
                KL.setPosition(CLOSEPOS);
            }
        }
        /*if (gamepad2.start) {
            leftS = false;
            rightS = false;
            isDpadL = false;
            isDpadR = false;
        }*/
        /*else { isDpadL = false; }
        if (gamepad2.dpad_right) {
            if (!isDpadR) {
                isDpadR = true;
                CLOSEPOS += 0.01;
                KL.setPosition(CLOSEPOS);
            }
        }
        else { isDpadR = false; }
    }

    void liftUp() {
        setLift(578);
    }

    double ROTPOS = 0;
    double DEFPOS  = 0.85;
    boolean isDpadU = false;
    boolean isDpadD = false;
    void servoControllerPro() {
        if (gamepad2.left_trigger > 0.6) {
            KL1.setPosition(DEFPOS);
        }
        if (gamepad2.right_trigger > 0.6) {
            KL1.setPosition(ROTPOS);
        }
        if (gamepad2.dpad_down) {
            if ( !isDpadD ) {
                isDpadD = true;
                OPENPOS -= 0.01;
                KL.setPosition(OPENPOS);
                //DEFPOS -= 0.025;
                //KL1.setPosition(DEFPOS);
            }
        } else { isDpadD = false; }
        if (gamepad2.dpad_up) {
            if ( !isDpadU ) {
                isDpadU = true;
                OPENPOS += 0.01;
                KL.setPosition(OPENPOS);
                //DEFPOS += 0.025;
                //KL1.setPosition(DEFPOS);
            }
        } else { isDpadU = false; }
    }

    void stabilizeKL() {
        KL1.setPosition(DEFPOS);
    }

    /*void liftUp() {
        LT.setPower(0.6);
        delay(1000);
    }*/

    /*void NESetPower(double power) {
        NE.setPower(Math.abs(power));
    }

    double plinkCount = 0;
    double plinkR = 0;
    double holdR = 0;
    void NeonController() {
        if ( NeState == "Driving" ) {
            double NePower = 0.3;
            NePower += Math.abs(gamepad1.left_stick_x)*0.2 + Math.abs(gamepad1.left_stick_y)*0.2 + Math.abs(gamepad1.right_stick_x)*0.2 + Math.abs(gamepad1.left_stick_y)*0.2 + Math.abs(gamepad1.left_trigger)*0.2 + Math.abs(gamepad1.right_trigger)*0.2;
            NESetPower(NePower);
        }
        if ( NeState == "Plink" ) {
            NESetPower((Math.abs(Math.sin(plinkR))*0.7)+0.2);
            plinkR += 0.01;
            if (plinkR >= 3) {
                plinkCount -= 1;
                plinkR = 0;
            }
            if (plinkCount == 0) {
                NeState = "Driving";
            }
        }
        if ( NeState == "Hold" ) {
            NESetPower(Math.abs(Math.sin(holdR)));
            holdR += 0.003;
        }
    }*/


    void g(double cc) { // функция езды вперёд на автономе
        telemetry = FtcDashboard.getInstance().getTelemetry(); // присваивание сути телеметрии из FTC Dashboard
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // обновление енкодер мотора
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // бег с использованием енкодера
        double ErLast = 0;
        double Ir = 0;
        double startRotation = getAngle();
        while ( Math.abs(cc - LF.getCurrentPosition()) > 5 && L.opModeIsActive()) {

            double Er = cc - LF.getCurrentPosition();

            //kp = 0.27
            //ki = 90
            //kd = 0.0005625

            double kp = 0.27;
            //telemetry.addData("KP", kp);
            double P = kp * Er ;

            double ki = 0.8;
            Ir += Er;
            double I = Ir * ki;

            double kd = 0003375;
            double ErD = ErLast - Er;
            double D = kd * ErD;

            if ( Math.abs(D) > Math.abs(P) ) { D=P; }

            //double REr = startRotation - getAngle();

            //double Rkp = 0.04;
            //double RP = Rkp * REr;

            //double RLF = -RP;
            //double RRB = -RP;

            //telemetry.addData("RP", RP);
            //telemetry.update();

            double pwf = P + I + D;

            telemetry.addData("LF", LF.getCurrentPosition());
            telemetry.addData("pwf", pwf);
            telemetry.update();

            LF.setPower(pwf);
            RB.setPower(-pwf);
        }
        setMtZero();
    }

    void rotate (double degrees) { //Функция автонома: поворот
        telemetry = FtcDashboard.getInstance().getTelemetry(); // присваивание сути телеметрии из FTC Dashboard
        double pw = 1;
        double Er0 = -degrees;
        double errorFix = 0;
        double ErLast = 0;
        double Ir = 0;
        if (Er0 > 0) {
            pw = -1;
        }
        degrees = getAngle() - degrees;
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
        while (Math.abs(degrees - getAngle()) > 3 && L.opModeIsActive()) {
            if (getAngle() > 0 && errorFix == 1) {
                Er0 = Er0 * -1;
                degrees += 360;
                pw *= -1;
                errorFix = 0;
            }
            if (getAngle() < 0 && errorFix == 2) {
                Er0 = Er0 * -1;
                degrees -= 360;
                pw *= -1;
                errorFix = 0;
            }

            double Er = degrees - (getAngle());

            double kp = 0.0012;
            double P = kp * Er / Er0 * pw;

            Ir += -Er;
            double ki = 1.2;
            double I = Ir * ki;

            double kd = 0.00003;
            double ErD = Er - ErLast;
            double D = kd * ErD * (1 / Er);

            if (Math.signum(D) > Math.signum(P)) {
                D = P;
            }

            double kr = -0.075;
            double Rele = kr * Math.signum(Er);

            ErLast = Er;


            double pwf = P + I + D; //Регулятор


            LB.setPower(pwf); // задание мощности
            LF.setPower(pwf);
            RB.setPower(pwf);
            RF.setPower(pwf);

                /*telemetry.addData("degrees", degrees);
                telemetry.addData("getAngle()", getAngle());
                telemetry.addData("Er (degrees - getAngle)", Er);
                telemetry.addData("Er0", Er0);
                telemetry.addData("kp", kp);
                telemetry.addData("P", P);
                telemetry.addData("D", D);
                telemetry.addData("Rele", Rele);
                telemetry.addData("pw", pw);
                telemetry.addData("pwf", pwf);
                telemetry.update();*/

            telemetry.addData("Angle", getAngle()); // добавление данных в телеметрию
            telemetry.addData("pwf", pwf);
            telemetry.update(); // обновление телеметрии

        }
    }

    double SMCLOSE = 0.6; // константы
    double SMOPEN = 1;
    double KLCLOSE = 0.12;
    double KLOPEN = 0.28;
    boolean isDLeft = false;
    boolean isDRight = false;
    void servoController() { // управление сервами в телеопе
        if (gamepad2.dpad_up ) {
            SM.setPosition(SMCLOSE);
        }
        // if (gamepad2.left_bumper) {
          //  KL.setPosition(KLCLOSE);
       // }
       // if (gamepad2.right_bumper) {
           // KL.setPosition(KLOPEN);
        //}
        /* if ( gamepad2.dpad_left ) {
            if ( ! isDLeft ) {
                KLCLOSE += 0.01;
                isDLeft = true;
                KL.setPosition(KLCLOSE);
            }
        }
        else {
            isDLeft = false;
        } */

       /* if ( gamepad2.dpad_right ) {
            if ( !isDRight ) {
                KLCLOSE -= 0.01;
                isDRight = true;
                KL.setPosition(KLCLOSE);
            }
        }
        else {
            isDRight = false;
        } */
    }

    double HOLDPOWER = 0.05;
    Thread liftControllerT = new Thread() { //Поток для лифта
        @Override
        public void run() { // основная функция потока
            boolean hold = false;
            double Power = 0;
            while (L.opModeIsActive() && !L.isStopRequested()) {
                //telemetry.addData("y", gamepad2.left_stick_y);
                //telemetry.update();
                LT.setPower((gamepad2.right_stick_y/-2.6769)+Power); //Управление лифтом стиком
                if (gamepad2.a) { // если нажата кнопка а
                    if ( hold ) { // условие держания лифта ?
                        Power = 0; // отключение силы лифта
                        hold = false; // опускаем флаг hold
                        delay(300); // задержка
                    }
                    else {
                        Power = HOLDPOWER; // значение силы, которая удержит лифт
                        hold = true; // поднимаем флаг hold
                        delay(300); // задержка
                    }
                }
                /*if (gamepad2.x) {
                    //NeState = "Plink";
                    //plinkCount = 1;
                    //setLift(240);
                    Power = HOLDPOWER;
                    hold = true;
                }
                if (gamepad2.b) {
                    //NeState = "Plink";
                    //plinkCount = 2;
                    //setLift(390);
                    Power = HOLDPOWER;
                    hold = true;
                }
                if (gamepad2.y) {
                    //NeState = "Plink";
                    //plinkCount = 3;
                    //setLift(545);
                    Power = HOLDPOWER;
                    hold = true;
                }*/
            }
        }
    };

}
