package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Reset TeleOp")
public class ResetTeleOp extends LinearOpMode {

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
                boolean pivotDone = Math.abs(distanceError) < 3.0;

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

                boolean liftDone = Math.abs(liftError) < 5;

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

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        boolean resetComplete = false;

        // Don't reset encoders, just set mode to RUN_WITHOUT_ENCODER
        robot.getPIVOT().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getCPIVOT().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getLIFT().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // First, reset everything to 0
        Actions.runBlocking(moveLift(robot, 0, 5.0));
        Actions.runBlocking(setWrist(robot, 0.0));
        Actions.runBlocking(movePivot(robot, 69, 5.0));

        resetComplete = true;

        while (opModeIsActive()) {
            if (resetComplete) {
                // Manual pivot control with gamepad2 left stick Y
                double pivotPower = gamepad2.left_stick_y; // Negate because negative is up on the gamepad
                robot.getPIVOT().setPower(pivotPower);
                robot.getCPIVOT().setPower(pivotPower);
            }

            // Telemetry
            telemetry.addData("Reset Complete", resetComplete);
            telemetry.addData("Pivot Position", robot.getPIVOT().getCurrentPosition());
            telemetry.addData("Lift Position", robot.getLIFT().getCurrentPosition());
            telemetry.addData("Pivot Power", robot.getPIVOT().getPower());
            telemetry.update();
        }
    }
}