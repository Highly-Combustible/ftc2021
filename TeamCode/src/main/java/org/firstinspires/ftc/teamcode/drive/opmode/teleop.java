package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

@TeleOp(name = "TeleOP", group = "teleOP")
public class teleop extends LinearOpMode {
    private DcMotor right_Front;
    private DcMotor right_Back;
    private DcMotor left_Front;
    private DcMotor left_Back;
    private DcMotor intake_Motor;
    private Servo index_Servo;
    private DcMotorEx firing_Motor;
    private Servo turn_Servo;
    private Servo claw_Servo;
    private autoTest autoTest;

    // This function is executed when this Op Mode is selected from the Driver Station.

    @Override
    public void runOpMode() {

        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        autoTest = new autoTest();

        drive.setPoseEstimate(new Pose2d(0, -37, Math.toRadians(180)));
        Pose2d shooterPos = new Pose2d(-3, -40, Math.toRadians(75));
        Pose2d powerShot = new Pose2d(-3, -24, Math.toRadians(75));

        right_Front = hardwareMap.get(DcMotor.class, "FrontRight");
        right_Back = hardwareMap.get(DcMotor.class, "BackRight");
        left_Front = hardwareMap.get(DcMotor.class, "FrontLeft");
        left_Back = hardwareMap.get(DcMotor.class, "BackLeft");
        intake_Motor = hardwareMap.get(DcMotor.class, "intakeMotor");
        firing_Motor = hardwareMap.get(DcMotorEx.class, "firingMotor");
        index_Servo = hardwareMap.servo.get("indexServo");
        turn_Servo = hardwareMap.servo.get("turnServo");
        claw_Servo = hardwareMap.servo.get("clawServo");

        //right_Front.setDirection(DcMotorSimple.Direction.REVERSE);
        //right_Back.setDirection(DcMotorSimple.Direction.REVERSE);
        //left_Front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_Back.setDirection(DcMotorSimple.Direction.REVERSE);
        firing_Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        firing_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
                    intake_Motor.setPower(1);
                }

                else if (gamepad1.y) {
                    intake_Motor.setPower (-0.6);
                }

                else {
                    intake_Motor.setPower(0);
                }

                if (gamepad1.x) {
                    autoTest.shoot(index_Servo, firing_Motor);
                }

                if (gamepad1.b) {
                    index_Servo.setPosition(0.4);
                }

                else {
                    index_Servo.setPosition(0.8);
                }

                if (gamepad1.right_bumper) {
                    firing_Motor.setVelocity((3800 * 28) / 60);
                }

                else if (gamepad1.left_bumper) {
                    firing_Motor.setVelocity((3300 * 28)/60);
                }

                else {
                    firing_Motor.setVelocity(0);
                }

                if (gamepad1.dpad_down) {
                    turn_Servo.setPosition(0.05);
                }

                else if (gamepad1.dpad_up){
                    turn_Servo.setPosition(0.6);
                }

                if (gamepad1.right_trigger >= 0.5) {
                    claw_Servo.setPosition(1);
                }

                else if (gamepad1.left_trigger >= 0.5) {
                    claw_Servo.setPosition(0.8);
                }

                if (gamepad1.dpad_left) {
                    drive.turn(Math.toRadians(8));
                }

                if (gamepad1.dpad_right) {
                     turn_Servo.setPosition(0.4);
                }

                /*  if (gamepad1.dpad_up) {
                    Trajectory teletraj1 = drive.trajectoryBuilder(currentPose)
                    .lineToSplineHeading(new Pose2d(shooterPos.getX(), shooterPos.getY(), shooterPos.getHeading()))
                    .build();

                    drive.followTrajectory(teletraj1);
                }

               if (gamepad1.back) {
                   Trajectory teletraj2 = drive.trajectoryBuilder(currentPose)
                   .lineToSplineHeading(new Pose2d(powerShot.getX(), powerShot.getY(), powerShot.getHeading()))
                   .build();

                   drive.followTrajectory(teletraj2);
               } */
            }
        }
    }
}
