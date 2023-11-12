/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "MecanumDrive")
@Config
public class MecanumDrive extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx LF = null;
    private DcMotorEx LB = null;
    private DcMotorEx RF = null;
    private DcMotorEx RB = null;
    private DcMotorEx intake = null;
    private DcMotorEx LeftSlide = null;
    private DcMotorEx RightSlide = null;
    private Servo liftin = null;

    public static double drivetrainSpeed = 0.7;
    public static double intakeSpeed = 0.8;
    public static int targetPosition = 0;
    public static int heightDiff = 20;
    public static int heightLimit = 820;
    public static double GoUpSpeed = 0.9;
    public static double GoDownSpeed = 0.55;
    public static double upPos = 1;
    public static double downPos = 0.6;
    private boolean intakeRunningForwards = true;
    private boolean intakeRunningBackwards = true;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LF = hardwareMap.get(DcMotorEx.class, "left_front");
        LB = hardwareMap.get(DcMotorEx.class, "left_back");
        RF = hardwareMap.get(DcMotorEx.class, "right_front");
        RB = hardwareMap.get(DcMotorEx.class, "right_back");

        liftin = hardwareMap.get(Servo.class, "liftin");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        LeftSlide = hardwareMap.get(DcMotorEx.class, "leftslide");
        RightSlide = hardwareMap.get(DcMotorEx.class, "rightslide");

        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);
        RightSlide.setDirection(DcMotorEx.Direction.FORWARD);
        LeftSlide.setDirection(DcMotorEx.Direction.REVERSE);
        liftin.setDirection(Servo.Direction.REVERSE);

        LF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        LeftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        LeftSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RightSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        LeftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RightSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        LF.setPower(frontLeftPower * drivetrainSpeed);
        LB.setPower(backLeftPower * drivetrainSpeed);
        RF.setPower(frontRightPower * drivetrainSpeed);
        RB.setPower(backRightPower * drivetrainSpeed);

        if (gamepad2.y && !gamepad2.a) {
            slidesUp();
        }

        if (gamepad2.a && !gamepad2.y) {
            slidesDown();
        }

        if (gamepad2.b && !gamepad2.x) {
            intake.setPower(Math.abs(intakeSpeed));
            intakeRunningForwards = true;
            intakeRunningBackwards = false;
        }

        if (gamepad2.x && !gamepad2.b) {
            intake.setPower(-intakeSpeed);
            intakeRunningForwards = false;
            intakeRunningBackwards = true;
        }

        if (gamepad2.left_bumper && gamepad2.right_bumper) {
            intake.setPower(0);
            intakeRunningForwards = false;
            intakeRunningBackwards = false;
        }

        if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            intakeSpeed = Math.min(intakeSpeed + 0.01, 1);
            if (intakeRunningForwards && !intakeRunningBackwards) {
                intake.setPower(intakeSpeed);
            } else if (intakeRunningBackwards && !intakeRunningForwards) {
                intake.setPower(-intakeSpeed);
            }
        }

        if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            intakeSpeed = Math.max(0, intakeSpeed - 0.01);
            if (intakeRunningForwards && !intakeRunningBackwards) {
                intake.setPower(intakeSpeed);
            } else if (intakeRunningBackwards && !intakeRunningForwards) {
                intake.setPower(-intakeSpeed);
            }
        }

        if (gamepad2.right_bumper && !gamepad2.left_bumper) {
            liftin.setPosition(upPos);
        }

        if (gamepad2.left_bumper && !gamepad2.right_bumper) {
            liftin.setPosition(downPos);
        }

        telemetry.addData("Runtime", runtime.toString());
        telemetry.addData("Intake Speed", String.valueOf(intakeSpeed));
        telemetry.addData("Drivetrain Speed", String.valueOf(drivetrainSpeed));
        telemetry.addData("Slides Position", String.valueOf(targetPosition));
        telemetry.addData("Slides Limit", String.valueOf(heightLimit));
        telemetry.addData("Slides Step", String.valueOf(heightDiff));
        telemetry.addData("Slides Extend Speed", String.valueOf(GoUpSpeed));
        telemetry.addData("Slides Retract Speed", String.valueOf(GoDownSpeed));
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void slidesUp() {
        targetPosition = Math.min(targetPosition + heightDiff, heightLimit);

        LeftSlide.setTargetPosition(targetPosition);
        RightSlide.setTargetPosition(targetPosition);

        LeftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        LeftSlide.setPower(GoUpSpeed);
        RightSlide.setPower(GoUpSpeed);
    }

    public void slidesDown() {
        targetPosition = Math.max(0, targetPosition - heightDiff);

        LeftSlide.setTargetPosition(targetPosition);
        RightSlide.setTargetPosition(targetPosition);

        LeftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        LeftSlide.setPower(GoDownSpeed);
        RightSlide.setPower(GoDownSpeed);
    }

}
