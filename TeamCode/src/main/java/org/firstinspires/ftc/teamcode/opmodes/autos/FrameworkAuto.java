package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "Framework Auto")
public class FrameworkAuto extends LinearOpMode {
    // Constants for arm positions
    private static final int RESET_PIVOT_POS = 0; // Matching the working Auto.java
    private static final int UP_PIVOT_POS = 3000;
    private static final int DOWN_PIVOT_POS = 100;

    private static final int UP_LIFT_POS = 3000;
    private static final int DOWN_LIFT_POS = 100;

    private static final double INTAKE_POWER = 1.0;
    private static final double OUTTAKE_POWER = -1.0;

    private Action moveToPosition(Robot robot, int targetPivot, int targetLift, double targetWrist, double timeout) {
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
                    robot.getLIFT().setPower(0.0);
                    return false;
                }

                int pivotError = targetPivot - robot.getPIVOT().getCurrentPosition();
                int liftError = targetLift - robot.getLIFT().getCurrentPosition();

                // Cap the power to valid range
                double pivotPower = pivotError * 0.02;
                pivotPower = Math.max(-1.0, Math.min(1.0, pivotPower));
                robot.getPIVOT().setPower(pivotPower);
                robot.getCPIVOT().setPower(pivotPower);

                double liftPower = liftError * 0.02;
                liftPower = Math.max(-1.0, Math.min(1.0, liftPower));
                robot.getLIFT().setPower(liftPower);

                robot.getWRIST().setPosition(targetWrist);

                p.put("pivotPos", robot.getPIVOT().getCurrentPosition());
                p.put("pivotPOWER", robot.getPIVOT().getPower());
                p.put("liftPos", robot.getLIFT().getCurrentPosition());
                p.put("liftPOWER", robot.getLIFT().getPower());
                p.put("ELAPSED TIME", elapsed);

                // Check if we've reached target positions
                boolean pivotDone = Math.abs(pivotError) < 200;
                boolean liftDone = Math.abs(liftError) < 200;

                if (pivotDone) robot.getPIVOT().setPower(0);robot.getCPIVOT().setPower(0);
                if (liftDone) robot.getLIFT().setPower(0);

                return !(pivotDone && liftDone);
            }
        };
    }

    private Action intakeAction(Robot robot, double power) {
        return new Action() {
            @Override
            public boolean run(TelemetryPacket p) {
                robot.getINTAKE().setPower(power);
                robot.getLINTAKE().setPower(-power);  // Note: opposite direction based on Auto.java
                sleep(1000);  // Same timing as Auto.java
                robot.getINTAKE().setPower(0);
                robot.getLINTAKE().setPower(0);
                return false;
            }
        };
    }

    @Override
    public void runOpMode() {
        // Initialize with the same starting position as Auto.java
        Pose2d startPose = new Pose2d(16.75/2, -(60+((double) (24-17)/2)), Math.toRadians(90.0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Robot robot = new Robot(hardwareMap);

        // Initialize motors
        robot.getPIVOT().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getLIFT().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getPIVOT().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getLIFT().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define trajectories (matching Auto.java)


        TrajectoryActionBuilder step1 = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(4.04, -38.8));



        // Reset pivot to starting position
        Actions.runBlocking(moveToPosition(robot, 0, 0, 0.0, 10.0));

        waitForStart();

        if (isStopRequested()) return;

        // Run the autonomous sequence
        Actions.runBlocking(new SequentialAction(
                step1.build(),
                moveToPosition(robot, 2260, 1374, 0.69, 8.0)

        ));

        while (opModeIsActive()) {
            // Display telemetry
            Pose2d poseEstimate = drive.localizer.getPose();
            telemetry.addData("x", poseEstimate.position.x);
            telemetry.addData("y", poseEstimate.position.y);
            telemetry.addData("heading", Math.toDegrees(poseEstimate.heading.toDouble()));
            telemetry.addData("Pivot Position", robot.getPIVOT().getCurrentPosition());
            telemetry.addData("Lift Position", robot.getLIFT().getCurrentPosition());
            telemetry.update();
        }
    }
}