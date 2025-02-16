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
    RESETTING,
    HANGING
}

@TeleOp(name = "TeleOp")
class NewTeleOP: LinearOpMode() {
    private companion object {
        const val ARM_DOWN_POSITION = 69.0 // mm when arm is fully down

        // Convert previous encoder positions to distance sensor readings
        const val INTAKE_PIVOT_DIST = 69.0  // mm (fully down)
        const val OUTTAKE_PIVOT_DIST = 250.0 // mm (estimated - adjust based on actual readings)
        const val RESET_PIVOT_DIST = 120.0  // mm (estimated - adjust based on actual readings)

        const val INTAKE_LIFT_POS = 0
        const val INTAKE_WRIST_POS = 0.8

        const val OUTTAKE_LIFT_POS = 2000
        const val OUTTAKE_WRIST_POS = 0.2

        const val RESET_LIFT_POS = 0
        const val RESET_WRIST_POS = 0.58

        const val HANGING_THRESHOLD = 10

        const val ENABLE_CONSTRAINTS = true
    }

    private fun updateManualControl(robot: Robot, gamepad: Gamepad): Triple<Double, Double, Double> {
        val PIVOT_MIN_DIST = ARM_DOWN_POSITION
        val PIVOT_MAX_DIST = 250.0  // mm - adjust based on maximum safe height
        val SLIDES_MAX = 1700.0
        val SLIDES_HOLD_THRESHOLD = 800

        var pivotPower = gamepad.left_stick_y.toDouble()
        var liftPower = gamepad.right_trigger.toDouble() - gamepad.left_trigger.toDouble()
        var intakePower = when {
            gamepad.right_bumper -> 1.0
            gamepad.left_bumper -> -1.0
            else -> 0.0
        }

        var wristDelta = gamepad.right_stick_y * 0.05
        robot.WRIST.position = (robot.WRIST.position - wristDelta).coerceIn(0.0, 1.0)

        if (ENABLE_CONSTRAINTS) {
            val currentDist = robot.PIV_DIST.getDistance(DistanceUnit.MM)

            if (currentDist >= PIVOT_MAX_DIST && pivotPower > 0) {
                pivotPower = 0.0
            } else if (currentDist <= PIVOT_MIN_DIST && pivotPower < 0) {
                pivotPower = 0.0
            }

            val distanceRatio = (currentDist - PIVOT_MIN_DIST) / (PIVOT_MAX_DIST - PIVOT_MIN_DIST)
            val maxExtension = SLIDES_MAX + lerp(0.0, 3000.0-SLIDES_MAX, distanceRatio)

            if (robot.LIFT.currentPosition > maxExtension && (liftPower > 0.0 || pivotPower < 0.0)) {
                liftPower = -0.1
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
        val wristError = abs(targetWrist - robot.WRIST.position)

        // Adjust power based on distance error
        val pivotPower = (distError * 0.02).coerceIn(-0.5, 0.5)
        robot.PIVOT.power = pivotPower
        robot.CPIVOT.power = pivotPower

        robot.LIFT.power = (liftError * 0.05).coerceIn(-0.8, 0.8)
        robot.WRIST.position = targetWrist

        return abs(distError) < 5 && abs(liftError) < 20 && wristError < 0.02
    }

    override fun runOpMode() {
        val timer = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val ROBOT = Robot(hardwareMap)
        val startPose =
            Pose2d(-24 - (16.75 / 2), -(60 + ((24 - 17).toDouble() / 2)), Math.toRadians(90.0))

        val LOCALIZER = MecanumDrive(hardwareMap, startPose)

        var m = 0.5

        ROBOT.PIVOT.power = 0.0
        ROBOT.CPIVOT.power = 0.0
        ROBOT.WRIST.position = 0.0

        var currentState = ArmState.MANUAL
        var lastIntakePower = 0.0

        telemetry.addData("Status", "Initialized")
        telemetry.addData("Constraints", if (ENABLE_CONSTRAINTS) "Enabled" else "Disabled")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("Loop Time", timer.milliseconds())
            timer.reset()

            when {
                gamepad2.dpad_up -> currentState = ArmState.MANUAL
                gamepad2.dpad_left -> currentState = ArmState.RESETTING
                gamepad2.dpad_right -> currentState = ArmState.OUTTAKING
                gamepad2.dpad_down -> currentState = ArmState.HANGING
            }

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

            when {
                gamepad1.cross -> m = 0.25
                gamepad1.circle -> m = 0.5
                gamepad1.square -> m = 0.75
                gamepad1.triangle -> m = 1.0
            }
            ROBOT.gamepadDrive(gamepad1, m)
            LOCALIZER.updatePoseEstimate()

            val POSE = LOCALIZER.localizer.pose

            telemetry.addData("Current State", currentState)
            telemetry.addData("Constraints", if (ENABLE_CONSTRAINTS) "Enabled" else "Disabled")
            telemetry.addData("Pivot Distance", ROBOT.PIV_DIST.getDistance(DistanceUnit.MM))
            telemetry.addData("Lift Position", ROBOT.LIFT.currentPosition)
            telemetry.addData("Wrist Position", ROBOT.WRIST.position)
            telemetry.addLine("")
            telemetry.addData("X: ", POSE.position.x)
            telemetry.addData("Y: ", POSE.position.y)
            telemetry.addData("HEADING: ", Math.toDegrees(POSE.heading.toDouble()))
            telemetry.update()

            val packet = TelemetryPacket()
            packet.fieldOverlay().setStroke("#3F51B5")
            Drawing.drawRobot(packet.fieldOverlay(), POSE)
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
        }
    }

    private fun lerp(a: Double, b: Double, t: Double): Double {
        return a + (b - a) * t
    }
}