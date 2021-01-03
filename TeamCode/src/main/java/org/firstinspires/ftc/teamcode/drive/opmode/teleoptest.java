
package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
@TeleOp(name = "intaketest", group = "teleOP")
public class teleoptest extends LinearOpMode {
    private DcMotor intake_motor;


    // This function is executed when this Op Mode is selected from the Driver Station.


    @Override
    public void runOpMode() {
        intake_motor = hardwareMap.dcMotor.get("intakeMotor");


        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {

                // speed that dpad controls go at; joy stick speed devided by this
                if (gamepad1.a) {
                    intake_motor.setPower(0.8);
                }
                else if (gamepad1.y) {
                    intake_motor.setPower(-0.8);
                }
                else {
                    intake_motor.setPower(0);
                }

            }
        }
    }
}