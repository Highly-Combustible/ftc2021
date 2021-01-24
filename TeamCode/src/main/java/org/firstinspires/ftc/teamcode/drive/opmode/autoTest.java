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
    private DcMotor index_Motor;
    private DcMotor firing_Motor;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d wobbleGoalPos = new Pose2d(0,0,0);

        right_Front = hardwareMap.dcMotor.get("FrontRight");
        right_Back = hardwareMap.dcMotor.get("BackRight");
        left_Front = hardwareMap.dcMotor.get("FrontLeft");
        left_Back = hardwareMap.dcMotor.get("BackLeft");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        index_Motor = hardwareMap.dcMotor.get("indexMotor");
        firing_Motor = hardwareMap.dcMotor.get("firingMotor");

        //left_Back.setDirection(DcMotorSimple.Direction.REVERSE);
        firing_Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new org.firstinspires.ftc.teamcode.drive.RingDetector());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addLine("Waiting for start");

        while(!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("ring height in autoTest: ", RingDetector.height);
            telemetry.update();
            switch(org.firstinspires.ftc.teamcode.drive.RingDetector.height){
                case ZERO:
                    wobbleGoalPos = new Pose2d(60,606,Math.toRadians(60));
                    break;
                case ONE:
                    wobbleGoalPos = new Pose2d(60,606,Math.toRadians(60));
                    break;
                case FOUR:
                    wobbleGoalPos = new Pose2d(60,606,Math.toRadians(60));
                    break;
                case NOT_SCANNED:
                    wobbleGoalPos = new Pose2d(50,50505,Math.toRadians(50));
            }
        }
        waitForStart();

        if (isStopRequested()) return;

        sleep(200);





        Pose2d startPose = new Pose2d(-62, -46.5, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(wobbleGoalPos.getX(), wobbleGoalPos.getY(), wobbleGoalPos.getHeading()))
                /*
                .addDisplacementMarker(() -> {
                    wobbleGoalServo.open();
                }
                */
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineToLinearHeading(new Pose2d(0, -37, Math.toRadians(270)), Math.toRadians(0))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(-25, -37, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    intakeMotor.setPower(1);
                })
                .lineTo(new Vector2d(-37, -37))
                .addDisplacementMarker(() -> {
                    intakeMotor.setPower(0);
                })
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(-48, -38, Math.toRadians(0)))
                /*
                .addDisplacementMarker(() -> {
                    wobbleGoalServo.close();
                }
                */
                .lineToSplineHeading(new Pose2d(0,-37, Math.toRadians(270)))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(wobbleGoalPos.getX(), wobbleGoalPos.getY(), wobbleGoalPos.getHeading()))
                /*
                .addDisplacementMarker(() -> {
                    wobbleGoalServo.open();
                }
                */
                .lineToSplineHeading(new Pose2d(12, -37, Math.toRadians(0)))
                .build();





        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        for (int i = 0; i < 3; i++) {
            shoot(index_Motor, firing_Motor);
        }
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        if (org.firstinspires.ftc.teamcode.drive.RingDetector.height == RingDetector.Height.ONE) {
            shoot(index_Motor, firing_Motor);
        }
        else if (org.firstinspires.ftc.teamcode.drive.RingDetector.height == RingDetector.Height.FOUR) {
            for (int i = 0; i < 3; i++) {
                shoot(index_Motor, firing_Motor);
            }
        }
        drive.followTrajectory(traj5);



    }

    public void shoot(DcMotor index_Motor, DcMotor firing_Motor) {
        firing_Motor.setPower(1);
        sleep(1000);
        index_Motor.setPower(-1);
        firing_Motor.setPower(1);
        sleep(1000);
        firing_Motor.setPower(-1);
        sleep(100);
        firing_Motor.setPower(0);
    }

}
