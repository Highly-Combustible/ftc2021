package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.RingDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class autoTest extends LinearOpMode {
    OpenCvCamera webcam;
    private DcMotor right_Front;
    private DcMotor right_Back;
    private DcMotor left_Front;
    private DcMotor left_Back;
    private DcMotor intakeMotor;
    private DcMotorEx firing_Motor;
    private Servo index_Servo;
    private Servo turn_Servo;
    private Servo claw_Servo;


    @Override
    public void runOpMode() throws InterruptedException {
        Vector2d wobbleGoalPos = new Vector2d(0, 0);

        right_Front = hardwareMap.get(DcMotor.class, "FrontRight");
        right_Back = hardwareMap.get(DcMotor.class, "BackRight");
        left_Front = hardwareMap.get(DcMotor.class, "FrontLeft");
        left_Back = hardwareMap.get(DcMotor.class, "BackLeft");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        firing_Motor = hardwareMap.get(DcMotorEx.class, "firingMotor");
        index_Servo = hardwareMap.servo.get("indexServo");
        turn_Servo = hardwareMap.servo.get("turnServo");
        claw_Servo = hardwareMap.servo.get("clawServo");

        //left_Back.setDirection(DcMotorSimple.Direction.REVERSE);
        firing_Motor.setDirection(DcMotorEx.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new org.firstinspires.ftc.teamcode.drive.RingDetector());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addLine("Waiting for start");

            while (!opModeIsActive() && !isStopRequested()) {
                telemetry.addData("ring height in autoTest: ", RingDetector.height);
                telemetry.update();
                switch (org.firstinspires.ftc.teamcode.drive.RingDetector.height) {
                    case ZERO:
                        wobbleGoalPos = new Vector2d(-30, -53);
                        break;
                    case ONE:
                        wobbleGoalPos = new Vector2d(36, -60);
                        break;
                    case FOUR:
                        wobbleGoalPos = new Vector2d(70, -45);
                        break;
                    case NOT_SCANNED:
                        wobbleGoalPos = new Vector2d(36, -25);
                }
            }
            waitForStart();

            if (isStopRequested()) return;

            sleep(200);


            Pose2d startPose = new Pose2d(-63, -24, Math.toRadians(0));
            webcam.stopStreaming();
            drive.setPoseEstimate(startPose);


        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(0, -24))
                /*
                .addDisplacementMarker(() -> {
                    wobbleGoalServo.open();
                }
                */
                .build();

            Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                    .lineToConstantHeading(new Vector2d(wobbleGoalPos.getX(), wobbleGoalPos.getY()))
                    /*
                    .addDisplacementMarker(() -> {
                        wobbleGoalServo.open();
                    }
                    */
                    .build();

            Trajectory traj1strafe = drive.trajectoryBuilder(traj1.end())
                    .strafeRight(10)
                    .build();

            Trajectory traj1back = drive.trajectoryBuilder(traj1strafe.end())
                    .back(10)
                    .build();

            Trajectory traj1goTo = drive.trajectoryBuilder(traj1back.end())
                    .lineTo(new Vector2d(0, -55))
                    .build();

            Trajectory traj2 = drive.trajectoryBuilder(traj1goTo.end())
                    .lineToSplineHeading(new Pose2d(0, -40, Math.toRadians(75)))
                    .build();

            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    .lineToSplineHeading(new Pose2d(-25, -37, Math.toRadians(180)))
                    /* .addDisplacementMarker(() -> {
                         intakeMotor.setPower(1);
                     })
                     .lineTo(new Vector2d(-37, -37))
                     .addDisplacementMarker(() -> {
                         intakeMotor.setPower(0);
                     }) */
                    .build();

            Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                    .lineToSplineHeading(new Pose2d(-48, -38, Math.toRadians(0)))
                    /*
                    .addDisplacementMarker(() -> {
                        wobbleGoalServo.close();
                    }
                    */
                    //.lineToSplineHeading(new Pose2d(0, -37, Math.toRadians(270)))
                    .build();

            Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                    .lineToSplineHeading(new Pose2d(wobbleGoalPos.getX(), wobbleGoalPos.getY(), Math.toRadians(0)))
                    /*
                    .addDisplacementMarker(() -> {
                        wobbleGoalServo.open();
                    }
                    */
                    //.lineToSplineHeading(new Pose2d(12, -37, Math.toRadians(0)))
                    .build();

            claw_Servo.setPosition(1);
            turn_Servo.setPosition(0.4);
            sleep(200);
            drive.followTrajectory(traj0);
            drive.followTrajectory(traj1);
            turn_Servo.setPosition(0.05);
            claw_Servo.setPosition(0.8);
            sleep(200);
            drive.followTrajectory(traj1strafe);
            sleep(200);
            drive.followTrajectory(traj1back);
            drive.followTrajectory(traj1goTo);
            drive.followTrajectory(traj2);
            chargeUp(firing_Motor);
            for (int i=0; i < 4; i++) { // Adjustment for servo weirdness
                flickServo(index_Servo, firing_Motor);
            }
            chargeDown(firing_Motor);
            //shoot(index_Servo, firing_Motor);
            //drive.followTrajectory(traj3);
            //drive.followTrajectory(traj4);
            if (RingDetector.height == RingDetector.Height.ONE || RingDetector.height == RingDetector.Height.FOUR) {
                    shoot(index_Servo, firing_Motor);
            }
            else if (RingDetector.height == RingDetector.Height.ZERO || RingDetector.height == RingDetector.Height.NOT_SCANNED) {
                intakeMotor.setPower(1);
                sleep(100);
            }
            //drive.followTrajectory(traj5);
        }


    public void shoot(Servo index_Servo, DcMotorEx firing_Motor) {
        firing_Motor.setVelocity((5000 * 28) / 60);;
        sleep(1500);
        for (int i = 0; i < 3; i++) {
            index_Servo.setPosition(0.4);
            firing_Motor.setVelocity((5000 * 28) / 60);
            sleep(200);
            index_Servo.setPosition(0.9);
            sleep(400);
            //telemetry.addLine("Shooting ring #:");
            //telemetry.addData("Shooting ring #: ", i);
            //telemetry.update();

        }
        firing_Motor.setPower(0);
        sleep(200);
        firing_Motor.setPower(-1);
        sleep(200);
        firing_Motor.setPower(0);
    }

    private void shootOnce(Servo index_Servo, DcMotorEx firing_Motor) {
            firing_Motor.setVelocity((5000 * 28) / 60);;
            sleep(1500);


            firing_Motor.setPower(0);
            sleep(200);
            firing_Motor.setPower(-1);
            sleep(200);
            firing_Motor.setPower(0);
    }

    private void flickServo(Servo index_Servo, DcMotorEx firing_Motor){
        index_Servo.setPosition(0.4);
        firing_Motor.setVelocity((5000 * 28) / 60);
        sleep(200);
        index_Servo.setPosition(0.9);
        sleep(400);
    }

    private void chargeUp(DcMotorEx firing_Motor)
    {
        firing_Motor.setVelocity((5000 * 28) / 60);;
        sleep(1500);
    }

    private void chargeDown(DcMotorEx firing_Motor){
        firing_Motor.setPower(0);
        sleep(200);
        firing_Motor.setPower(-1);
        sleep(200);
        firing_Motor.setPower(0);
    }

}
