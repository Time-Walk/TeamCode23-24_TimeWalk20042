package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="тест распознование ааааа я спать хочу", group="")
public class testDetection extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023();
        R.initFields(telemetry, this, hardwareMap);
        R.init();

        DetectionPipeLine pipeLine = new DetectionPipeLine();
        pipeLine.targetColor = "RED";
        pipeLine.initFixed(telemetry);
        R.camera.setPipeline(pipeLine);
        R.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                R.camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard dash = FtcDashboard.getInstance();
                dash.startCameraStream(R.camera, 30);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("result", pipeLine.getResult());
            R.delay(500);
        }

        //okay, let's go!

        telemetry.addData("result", pipeLine.getResult());
        telemetry.update();

    }

}
