package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group="Tests")
public class ChassisDriverTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("left_front");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("left_back");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("right_front");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("right_back");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        double speedDivisorDouble = 3;

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            int speedDivisorInt = (int) Math.floor(speedDivisorDouble);
            if (gamepad1.left_stick_button){
                switch(speedDivisorInt){
                    case 3:
                        speedDivisorInt = 1;
                        break;
                    case 2:
                        speedDivisorInt = 3;
                        break;
                    case 1:
                        speedDivisorInt = 2;
                        break;
                }
            }

            if (gamepad1.left_bumper) {
                frontLeftMotor.setPower(-(1 / speedDivisorDouble));
                backRightMotor.setPower(-(1 / speedDivisorDouble) * 1.1);
                frontRightMotor.setPower(1 / speedDivisorDouble);
                backLeftMotor.setPower((1 / speedDivisorDouble));
            }else if (gamepad1.right_bumper){
                frontLeftMotor.setPower((1/speedDivisorDouble));
                backRightMotor.setPower((1/speedDivisorDouble));
                frontRightMotor.setPower(-(1/speedDivisorDouble) * 1.1);
                backLeftMotor.setPower(-(1/speedDivisorDouble));
            }else {


                double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = -(gamepad1.left_stick_x * 1.1); // Counteract imperfect strafing
                double rx = (-gamepad1.right_stick_x);

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower / speedDivisorDouble);
                backLeftMotor.setPower(backLeftPower / speedDivisorDouble);
                frontRightMotor.setPower(frontRightPower / speedDivisorDouble);
                backRightMotor.setPower(backRightPower / speedDivisorDouble);
            }
        }
    }
}