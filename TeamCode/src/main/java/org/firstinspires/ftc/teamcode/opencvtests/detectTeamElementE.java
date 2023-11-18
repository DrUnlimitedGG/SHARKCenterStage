package org.firstinspires.ftc.teamcode.opencvtests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencvtests.blueTeamElementDetector;
import org.firstinspires.ftc.teamcode.opencvtests.RedTeamElementDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Team Element Detector")
public class detectTeamElementE extends LinearOpMode {

    private blueTeamElementDetector blueDetector;
    private RedTeamElementDetector redDetector;
    boolean isRed = true;
    private OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        if(isRed)
        {
            redDetector = new RedTeamElementDetector();
            camera.setPipeline(redDetector);
        }
        else
        {
            blueDetector = new blueTeamElementDetector();
            camera.setPipeline(blueDetector);
        }

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            if(isRed) {
                telemetry.addData("POSITION: ", RedTeamElementDetector.getPosition());
                telemetry.addData("RIGHT_AREA_COUNT: ", RedTeamElementDetector.rightAreaCount());
                telemetry.addData("MIDDLE_AREA_COUNT: ", RedTeamElementDetector.middleAreaCount());
                telemetry.addData("COLOR: ", "RED");
            }
            else
            {
                telemetry.addData("POSITION: ", blueTeamElementDetector.getPosition());
                telemetry.addData("RIGHT_AREA_COUNT: ", blueTeamElementDetector.rightAreaCount());
                telemetry.addData("MIDDLE_AREA_COUNT: ", blueTeamElementDetector.middleAreaCount());
                telemetry.addData("COLOR: ", "BLUE");
            }
            if(gamepad1.x)
            {
                isRed = false;
            }
            else if (gamepad1.y)
            {
                isRed = true;
            }
            telemetry.update();
        }
        waitForStart();


    }
}