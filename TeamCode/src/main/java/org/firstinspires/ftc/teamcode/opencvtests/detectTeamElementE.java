package org.firstinspires.ftc.teamcode.opencvtests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opencvtests.blueTeamElementDetector;
import org.firstinspires.ftc.teamcode.opencvtests.RedTeamElementDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Team Element Detector")
public class detectTeamElementE extends LinearOpMode {

    private blueTeamElementDetector blueDetector;
    private RedTeamElementDetector redDetector;
    boolean isRed = true;
    private OpenCvCamera camera;
    int position;

    private DcMotorEx intake = null;



    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.get(DcMotorEx .class, "intake");

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(-90));


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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
        if (!isStopRequested())


            // put the code here
        // if it's red, do if (isRed)
        // then inside do if RedTeamElementDetector.getPosition() == integer
        if(isRed)
        {
           position = RedTeamElementDetector.getPosition();
        }
        else
        {
            position = blueTeamElementDetector.getPosition();
        }

        TrajectorySequence forward = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(25, 0))
                .build();
        TrajectorySequence rightAdjust = drive.trajectorySequenceBuilder(new Pose2d(25,0,0))
        .lineToConstantHeading(new Vector2d(25, 10))
                .turn(Math.toRadians(90))
                .build();
        TrajectorySequence leftAdjust = drive.trajectorySequenceBuilder(new Pose2d(25,0,0))
                .lineToConstantHeading(new Vector2d(25, -10))
                .turn(Math.toRadians(-90))
                .build();
        drive.followTrajectorySequence(forward);

                if (position == 1) {

                    drive.followTrajectorySequence(leftAdjust);

                }

                if (position == 3) {
                    drive.followTrajectorySequence(rightAdjust);

                }
                intake.setPower(-0.3);
                Thread.sleep(1000);
                intake.setPower(0);




    }
}