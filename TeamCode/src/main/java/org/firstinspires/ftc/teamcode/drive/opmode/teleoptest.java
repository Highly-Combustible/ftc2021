package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

@TeleOp(name = "TeleOP Test", group = "teleOP")
public class teleoptest extends LinearOpMode {
    private DcMotor right_Front;
    private DcMotor right_Back;
    private DcMotor left_Front;
    private DcMotor left_Back;
    private DcMotor intake_Motor;
    private Servo index_Servo;
    private DcMotor firing_Motor;
    private Servo turn_Servo;
    private Servo claw_Servo;
    private autoTest autoTest;

    // This function is executed when this Op Mode is selected from the Driver Station.

    @Override
    public void runOpMode() {

        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        autoTest = new autoTest();

        myLocalizer.setPoseEstimate(new Pose2d(12, -37, Math.toRadians(0)));

        Pose2d shooterPos = new Pose2d(12, -37, Math.toRadians(300));

        right_Front = hardwareMap.dcMotor.get("FrontRight");
        right_Back = hardwareMap.dcMotor.get("BackRight");
        left_Front = hardwareMap.dcMotor.get("FrontLeft");
        left_Back = hardwareMap.dcMotor.get("BackLeft");
        intake_Motor = hardwareMap.dcMotor.get("intakeMotor");
        index_Servo = hardwareMap.servo.get("indexServo");
        firing_Motor = hardwareMap.dcMotor.get("firingMotor");
        turn_Servo = hardwareMap.servo.get("turnServo");
        claw_Servo = hardwareMap.servo.get("clawServo");

        //right_Front.setDirection(DcMotorSimple.Direction.REVERSE);
        //right_Back.setDirection(DcMotorSimple.Direction.REVERSE);
        //left_Front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_Back.setDirection(DcMotorSimple.Direction.REVERSE);
        firing_Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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

                Pose2d currentPose = drive.getPoseEstimate();
                drive.update();

                // speed that dpad controls go at; joy stick speed divided by this
                if (gamepad1.a) {
                    intake_Motor.setPower(0.8);
                }

                else if (gamepad1.y) {
                    intake_Motor.setPower (-0.8);
                }

                else {
                    intake_Motor.setPower(0);
                }
/*
                if (gamepad1.x) {
                    autoTest.shoot(index_Motor, firing_Motor);
                }
*/
                if (gamepad1.dpad_down) {
                    index_Servo.setPosition(1);
                }

                else if (gamepad1.b) {
                    index_Servo.setPosition(0.4);
                }

                else {
                    index_Servo.setPosition(0.8);
                }
             /*   else {
                    index_Servo.setPower(0);
                }
*/
                if (gamepad1.right_bumper) {
                    firing_Motor.setPower(1);
                }

                else {
                    firing_Motor.setPower(0);
                }

                if (gamepad1.dpad_left) {
                    turn_Servo.setPosition(0.1);
                }

                else if (gamepad1.dpad_right){
                    turn_Servo.setPosition(0.4);
                }

                if (gamepad1.right_trigger >= 1) {
                    claw_Servo.setPosition(0.95);
                }

                else if (gamepad1.left_trigger >= 1) {
                    claw_Servo.setPosition(0.8);
                }

                if (gamepad1.dpad_up) {
                    Trajectory teletraj1 = drive.trajectoryBuilder(currentPose)
                    .lineToSplineHeading(new Pose2d(shooterPos.getX(), shooterPos.getY(), shooterPos.getHeading()))
                    .build();
                }

            }
        }
    }
}
