package org.firstinspires.ftc.teamcode.opmodes.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "Specimen")
public class FrameworkAuto extends LinearOpMode {
    // Constants for arm positions in millimeters
    private static final double ARM_DOWN_POSITION = 72.0; // mm when arm is fully down
    private static final double RESET_PIVOT_DIST = ARM_DOWN_POSITION;
    private static final double UP_PIVOT_DIST = (121 - 69) + ARM_DOWN_POSITION - 5; // Adjust based on actual measurement
    private static final double DOWN_PIVOT_DIST = 90.0; // Slightly above fully down

    private static final int UP_LIFT_POS = 3000;
    private static final int DOWN_LIFT_POS = 100;

    private static final double INTAKE_POWER = 1.0;
    private static final double OUTTAKE_POWER = -1.0;

    private double getCurrentPivotDistance(Robot robot) {
        return robot.getPIV_DIST().getDistance(DistanceUnit.MM);
    }

    private Action movePivot(Robot robot, double targetDistance, double timeout) {
        return new Action() {
            private boolean initialized = false;
            private long startTime;

            @Override
            public boolean run(TelemetryPacket p) {
                if (!initialized) {
                    startTime = System.nanoTime();
                    initialized = true;
                }

                double elapsed = (System.nanoTime() - startTime) / 1e9;
                if (elapsed > timeout) {
                    robot.getPIVOT().setPower(0.0);
                    robot.getCPIVOT().setPower(0.0);
                    return false;
                }

                double currentDistance = getCurrentPivotDistance(robot);
                double distanceError = targetDistance - currentDistance;

                // Adjust power calculation for distance-based control
                double pivotPower = distanceError * 0.1;
                pivotPower = Math.max(-1.0, Math.min(1.0, pivotPower));
                robot.getPIVOT().setPower(pivotPower);
                robot.getCPIVOT().setPower(pivotPower);

                p.put("currentDistance", currentDistance);
                p.put("targetDistance", targetDistance);
                p.put("distanceError", distanceError);
                p.put("pivotPower", pivotPower);
                p.put("ELAPSED TIME", elapsed);

                // Using 3mm tolerance as requested
                boolean pivotDone = Math.abs(distanceError) < 2.0;

                if (pivotDone) {
                    robot.getPIVOT().setPower(0);
                    robot.getCPIVOT().setPower(0);
                }

                return !pivotDone;
            }
        };
    }

    private Action moveLift(Robot robot, int targetLift, double timeout) {
        return new Action() {
            private boolean initialized = false;
            private long startTime;

            @Override
            public boolean run(TelemetryPacket p) {
                if (!initialized) {
                    startTime = System.nanoTime();
                    initialized = true;
                }

                double elapsed = (System.nanoTime() - startTime) / 1e9;
                if (elapsed > timeout) {
                    robot.getLIFT().setPower(0.0);
                    return false;
                }

                int liftError = targetLift - robot.getLIFT().getCurrentPosition();

                double liftPower = liftError * 0.02;
                liftPower = Math.max(-1.0, Math.min(1.0, liftPower));
                robot.getLIFT().setPower(liftPower);

                p.put("liftPos", robot.getLIFT().getCurrentPosition());
                p.put("liftPower", liftPower);
                p.put("ELAPSED TIME", elapsed);

                boolean liftDone = Math.abs(liftError) < 10;

                if (liftDone) robot.getLIFT().setPower(0);

                return !liftDone;
            }
        };
    }

    private Action setWrist(Robot robot, double targetWrist) {
        return new Action() {
            @Override
            public boolean run(TelemetryPacket p) {
                robot.getWRIST().setPosition(targetWrist);
                return false;
            }
        };
    }

    private Action setIntakePower(Robot robot, double targetPower) {
        return new Action() {
            @Override
            public boolean run (TelemetryPacket p) {
                robot.getINTAKE().setPower(targetPower);
                robot.getLINTAKE().setPower(targetPower);
                return false;
            }
        };
    }

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(16.75/2, -(60+((double) (24-17)/2)), Math.toRadians(90.0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Robot robot = new Robot(hardwareMap);

        // Initialize motors - no need to reset encoders for pivot
        robot.getLIFT().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getLIFT().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set initial motor powers to zero
        robot.getPIVOT().setPower(0);
        robot.getCPIVOT().setPower(0);

        TrajectoryActionBuilder step1 = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(4.04, -38.8));


        TrajectoryActionBuilder step2 = step1.endTrajectory().fresh()
                .strafeTo(new Vector2d(37.5, -38))
                .strafeTo(new Vector2d(37.5, -14.6))
                .strafeToLinearHeading(new Vector2d(47.6, -14.6), 0)
                .strafeTo(new Vector2d(45.5, -57.6))

                .strafeTo(new Vector2d(45.5, -47))

                .strafeToLinearHeading(new Vector2d( 48, -45), Math.toRadians(-90));

        TrajectoryActionBuilder step3 = step2.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, -52.4));

        TrajectoryActionBuilder step4 = step3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(6.04, -38.8), Math.toRadians(90));

        // Reset to starting position
        Actions.runBlocking(new SequentialAction(
                movePivot(robot, ARM_DOWN_POSITION, 5.0),
                moveLift(robot, 0, 5.0),
                setWrist(robot, 0.0)
        ));

        waitForStart();

        if (isStopRequested()) return;

        // Run the autonomous sequence
        Actions.runBlocking(new SequentialAction(
                movePivot(robot, DOWN_PIVOT_DIST, 4.0), // Adjusted from 600 encoder ticks to estimated distance
                step1.build(),
                movePivot(robot, UP_PIVOT_DIST, 4.0),
                setWrist(robot, 0.46),
                new SleepAction(1.0),
                moveLift(robot, 1380, 4.0),

                moveLift(robot, 0, 4.0),
                setWrist(robot, 0),
                movePivot(robot, RESET_PIVOT_DIST, 4.0),
                setIntakePower(robot, -1),
                new SleepAction(0.5),
                setIntakePower(robot, 0),

                step2.build(),
                new SleepAction(3),

                movePivot(robot, 70, 4.0),
                setWrist(robot, 0.395),
                step3.build(),
                new SleepAction(0.5),
                setIntakePower(robot, 1),
                moveLift(robot, 800, 4.0),
                new SleepAction(0.5),

                setIntakePower(robot, 0),
                moveLift(robot, 0, 4.0),
                setWrist(robot, 0),
                movePivot(robot, RESET_PIVOT_DIST, 4.0),



                movePivot(robot, DOWN_PIVOT_DIST, 4.0), // Adjusted from 600 encoder ticks to estimated distance
                step4.build(),
                movePivot(robot, UP_PIVOT_DIST, 4.0),
                setWrist(robot, 0.46),
                new SleepAction(1.0),
                moveLift(robot, 1380, 4.0),

                moveLift(robot, 0, 4.0),
                setWrist(robot, 0),
                movePivot(robot, RESET_PIVOT_DIST, 4.0)

        ));

        while (opModeIsActive()) {
            // Display telemetry
            Pose2d poseEstimate = drive.localizer.getPose();
            telemetry.addData("x", poseEstimate.position.x);
            telemetry.addData("y", poseEstimate.position.y);
            telemetry.addData("heading", Math.toDegrees(poseEstimate.heading.toDouble()));
            telemetry.addData("Pivot Distance (mm)", getCurrentPivotDistance(robot));
            telemetry.addData("Lift Position", robot.getLIFT().getCurrentPosition());
            telemetry.update();
        }
    }
}