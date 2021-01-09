
package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
@TeleOp(name = "TeleOP Test", group = "teleOP")
public class teleoptest extends LinearOpMode {
    private DcMotor right_Front;
    private DcMotor right_Back;
    private DcMotor left_Front;
    private DcMotor left_Back;
    private DcMotor intake_motor;

    // This function is executed when this Op Mode is selected from the Driver Station.

    @Override
    public void runOpMode() {

        right_Front = hardwareMap.dcMotor.get("FrontRight");
        right_Back = hardwareMap.dcMotor.get("BackRight");
        left_Front = hardwareMap.dcMotor.get("FrontLeft");
        left_Back = hardwareMap.dcMotor.get("BackLeft");
        intake_motor = hardwareMap.dcMotor.get("intakeMotor");

        //right_Front.setDirection(DcMotorSimple.Direction.REVERSE);
        //right_Back.setDirection(DcMotorSimple.Direction.REVERSE);
        //left_Front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_Back.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {

                double LPY = -gamepad1.left_stick_y;
                double LPX = gamepad1.left_stick_x;
                double RPX = gamepad1.right_stick_x;
                
                right_Front.setPower(LPY - LPX - RPX);
                right_Back.setPower(LPY + LPX - RPX);
                left_Front.setPower(LPY + LPX + RPX);
                left_Back.setPower(LPY - LPX + RPX);

                right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                right_Back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                left_Back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


                // speed that dpad controls go at; joy stick speed devided by this
                if (gamepad1.a) {
                    intake_motor.setPower(1);
                }

                else if (gamepad1.y) {
                    intake_motor.setPower (-1);
                }

                else {
                    intake_motor.setPower(0);
                }

            }
        }
    }
}