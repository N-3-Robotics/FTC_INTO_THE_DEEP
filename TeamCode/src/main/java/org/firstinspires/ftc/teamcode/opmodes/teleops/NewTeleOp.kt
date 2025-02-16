package org.firstinspires.ftc.teamcode.opmodes.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.Drawing
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.robot.Robot
import kotlin.math.abs

private enum class ArmState {
    MANUAL,
    INTAKING,
    OUTTAKING,
    SAMPLE,
    RESETTING,
    HANGING
}

@TeleOp(name = "TeleOp")
class NewTeleOP: LinearOpMode() {
    private companion object {
        // Distance sensor constants (in mm)
        const val MIN_PIVOT_DIST = 72.0  // Minimum safe distance
        const val MAX_PIVOT_DIST = 150.0 // Maximum safe distance

        // Preset positions using distance sensor values
        const val INTAKE_PIVOT_DIST = 72.0  // Fully down for intake
        const val OUTTAKE_PIVOT_DIST = 145.0 // Position for outtaking
        const val RESET_PIVOT_DIST = 72.0  // Reset position

        // Lift positions (using encoder values)
        const val INTAKE_LIFT_POS = 0
        const val OUTTAKE_LIFT_POS = 2910
        const val RESET_LIFT_POS = 0

        // Wrist positions (servo values 0-1)
        const val INTAKE_WRIST_POS = 0.38
        const val OUTTAKE_WRIST_POS = 0.955
        const val RESET_WRIST_POS = 0.0

        const val HANGING_THRESHOLD = 10
        const val ENABLE_CONSTRAINTS = true
    }

    private fun updateManualControl(robot: Robot, gamepad: Gamepad): Triple<Double, Double, Double> {
        // Get current pivot position from distance sensor
        val currentPivotDist = robot.PIV_DIST.getDistance(DistanceUnit.MM)

        var pivotPower = gamepad.left_stick_y.toDouble() // Invert for intuitive control
        var liftPower = gamepad.right_trigger.toDouble() - gamepad.left_trigger.toDouble()
        var intakePower = when {
            gamepad.right_bumper -> 1.0
            gamepad.left_bumper -> -1.0
            else -> 0.0
        }

        // Update wrist position
        var wristDelta = -gamepad.right_stick_y * 0.05
        robot.WRIST.position = (robot.WRIST.position + wristDelta).coerceIn(0.0, 1.0)

        if (ENABLE_CONSTRAINTS) {
            // Prevent pivot from going beyond safe limits
            if (currentPivotDist >= MAX_PIVOT_DIST && pivotPower > 0) {
                pivotPower = 0.0
            } else if (currentPivotDist <= MIN_PIVOT_DIST && pivotPower < 0) {
                pivotPower = 0.0
            }

            // Slide limits based on pivot position
            val pivotRatio = (currentPivotDist - MIN_PIVOT_DIST) / (MAX_PIVOT_DIST - MIN_PIVOT_DIST)
            val maxSlideExtension = lerp(1700.0, 3300.0, pivotRatio)

            if (robot.LIFT.currentPosition > maxSlideExtension && liftPower > 0) {
                liftPower = 0.0
            } else if (robot.LIFT.currentPosition < 0 && liftPower < 0) {
                liftPower = 0.0
            }
        }

        return Triple(pivotPower, liftPower, intakePower)
    }

    private fun moveToPosition(robot: Robot, targetDist: Double, targetLift: Int, targetWrist: Double): Boolean {
        val currentDist = robot.PIV_DIST.getDistance(DistanceUnit.MM)
        val distError = targetDist - currentDist
        val liftError = targetLift - robot.LIFT.currentPosition

        // Calculate pivot power without holding component
        val pivotPower = (distError * 0.1).coerceIn(-0.5, 0.5)

        robot.PIVOT.power = pivotPower
        robot.CPIVOT.power = pivotPower

        // Calculate lift power
        val liftPower = (liftError * 0.02).coerceIn(-0.8, 0.8)
        robot.LIFT.power = liftPower

        // Update wrist position
        robot.WRIST.position = targetWrist

        // Return true when position is reached
        return abs(distError) < 2.0 && abs(liftError) < 30
    }

    override fun runOpMode() {
        val timer = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val ROBOT = Robot(hardwareMap)
        val startPose = Pose2d(-24 - (16.75 / 2), -(60 + ((24 - 17).toDouble() / 2)), Math.toRadians(90.0))
        val LOCALIZER = MecanumDrive(hardwareMap, startPose)

        var driveSpeed = 0.5
        var currentState = ArmState.MANUAL
        var lastIntakePower = 0.0

        ROBOT.PIVOT.power = 0.0
        ROBOT.CPIVOT.power = 0.0
        ROBOT.WRIST.position = RESET_WRIST_POS

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("Loop Time", timer.milliseconds())
            timer.reset()

            // State transitions
            when {
                gamepad2.dpad_up -> currentState = ArmState.MANUAL
                gamepad2.dpad_left -> currentState = ArmState.RESETTING
                gamepad2.dpad_right -> currentState = ArmState.OUTTAKING
                gamepad2.dpad_down -> currentState = ArmState.HANGING
                gamepad2.triangle -> currentState = ArmState.SAMPLE
            }

            // State machine
            when (currentState) {
                ArmState.MANUAL -> {
                    val (pivotPower, liftPower, intakePower) = updateManualControl(ROBOT, gamepad2)
                    ROBOT.PIVOT.power = pivotPower
                    ROBOT.CPIVOT.power = pivotPower
                    ROBOT.LIFT.power = liftPower

                    if (intakePower != lastIntakePower) {
                        ROBOT.INTAKE.power = intakePower
                        ROBOT.LINTAKE.power = intakePower
                        lastIntakePower = intakePower
                    }
                }

                ArmState.RESETTING -> {
                    if (moveToPosition(ROBOT, RESET_PIVOT_DIST, RESET_LIFT_POS, RESET_WRIST_POS)) {
                        currentState = ArmState.MANUAL
                    }
                }

                ArmState.INTAKING -> {
                    moveToPosition(ROBOT, INTAKE_PIVOT_DIST, INTAKE_LIFT_POS, INTAKE_WRIST_POS)
                    ROBOT.INTAKE.power = 1.0
                    ROBOT.LINTAKE.power = 1.0
                }

                ArmState.SAMPLE -> {
                    if (moveToPosition(ROBOT, 70.0, 0, 0.395)) {
                        currentState = ArmState.MANUAL
                    }
                }

                ArmState.OUTTAKING -> {
                    moveToPosition(ROBOT, OUTTAKE_PIVOT_DIST, OUTTAKE_LIFT_POS, OUTTAKE_WRIST_POS)
                    ROBOT.INTAKE.power = -1.0
                    ROBOT.LINTAKE.power = -1.0
                }

                ArmState.HANGING -> {
                    if (abs(ROBOT.LIFT.currentPosition) > HANGING_THRESHOLD) {
                        ROBOT.LIFT.power = -0.5
                        ROBOT.PIVOT.power = 0.0
                        ROBOT.CPIVOT.power = 0.0
                    } else {
                        ROBOT.LIFT.power = 0.0
                    }
                }
            }

            // Drive speed control
            when {
                gamepad1.cross -> driveSpeed = 0.25
                gamepad1.circle -> driveSpeed = 0.5
                gamepad1.square -> driveSpeed = 0.75
                gamepad1.triangle -> driveSpeed = 1.0
            }

            ROBOT.gamepadDrive(gamepad1, driveSpeed)
            LOCALIZER.updatePoseEstimate()

            // Telemetry
            val pose = LOCALIZER.localizer.pose
            telemetry.addData("Current State", currentState)
            telemetry.addData("Pivot Distance (mm)", ROBOT.PIV_DIST.getDistance(DistanceUnit.MM))
            telemetry.addData("Lift Position", ROBOT.LIFT.currentPosition)
            telemetry.addData("Wrist Position", ROBOT.WRIST.position)
            telemetry.addData("Drive Speed", driveSpeed)
            telemetry.addLine()
            telemetry.addData("X", pose.position.x)
            telemetry.addData("Y", pose.position.y)
            telemetry.addData("Heading", Math.toDegrees(pose.heading.toDouble()))
            telemetry.update()

            // Dashboard visualization
            val packet = TelemetryPacket()
            packet.fieldOverlay().setStroke("#3F51B5")
            Drawing.drawRobot(packet.fieldOverlay(), pose)
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
        }
    }

    private fun lerp(a: Double, b: Double, t: Double): Double {
        return a + (b - a) * t
    }
}