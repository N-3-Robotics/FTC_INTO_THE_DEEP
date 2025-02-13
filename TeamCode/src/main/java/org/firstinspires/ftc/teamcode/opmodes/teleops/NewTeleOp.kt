package org.firstinspires.ftc.teamcode.opmodes.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robot.Robot
import kotlin.math.abs

private enum class ArmState {
    MANUAL,     // Manual control of all arm functions
    INTAKING,   // Automatic intaking position
    OUTTAKING,  // Automatic outtaking position
    HANGING     // Hanging state for endgame
}

@TeleOp(name = "TeleOp")
class NewTeleOP: LinearOpMode() {
    // Constants for automatic positions
    private companion object {
        const val INTAKE_PIVOT_POS = 0
        const val INTAKE_LIFT_POS = 0
        const val INTAKE_WRIST_POS = 0.8

        const val OUTTAKE_PIVOT_POS = 2800
        const val OUTTAKE_LIFT_POS = 2000
        const val OUTTAKE_WRIST_POS = 0.2

        const val HANGING_THRESHOLD = 10 // Margin for considering slides fully retracted

        // Toggle for enabling/disabling constraints
        const val ENABLE_CONSTRAINTS = true
    }

    private fun resetEncoders(robot: Robot) {
        // Reset all encoders
        robot.PIVOT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        robot.CPIVOT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        robot.LIFT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        // Set back to run mode
        robot.PIVOT.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        robot.CPIVOT.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        robot.LIFT.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    private fun updateManualControl(robot: Robot, gamepad: Gamepad): Triple<Double, Double, Double> {
        val PIVOT_MAX = 3386.0
        val SLIDES_MAX = 1700.0
        val SLIDES_HOLD_THRESHOLD = 800 // New constant for slides hold position

        var pivotPower = gamepad.left_stick_y.toDouble()
        var liftPower = -gamepad.right_stick_y.toDouble()
        var intakePower = when {
            gamepad.right_bumper -> 1.0
            gamepad.left_bumper -> -1.0
            else -> 0.0
        }

        // Apply safety limits only if constraints are enabled
        if (ENABLE_CONSTRAINTS) {
            val pivotPOS = robot.PIVOT.currentPosition
            if (pivotPOS >= PIVOT_MAX && pivotPower > 0) {
                pivotPower = 0.0
            } else if (pivotPOS < 0 && pivotPower < 0) {
                pivotPower = 0.0
            }

            val maxExtension = SLIDES_MAX + lerp(0.0, 3000.0-SLIDES_MAX, (pivotPOS / PIVOT_MAX))
            if (robot.LIFT.currentPosition > maxExtension && (liftPower > 0.0 || pivotPower < 0.0)) {
                liftPower = -0.1
            } else if (robot.LIFT.currentPosition < 0 && liftPower < 0) {
                liftPower = 0.0
            }

            // Add holding power when slides are extended past threshold
            if (robot.LIFT.currentPosition > SLIDES_HOLD_THRESHOLD && liftPower == 0.0) {
                liftPower = 0.1
            }
        }

        return Triple(pivotPower, liftPower, intakePower)
    }

    private fun moveToPosition(robot: Robot, targetPivot: Int, targetLift: Int, targetWrist: Double) {
        // Simple position control - you might want to implement PID control here
        val pivotError = targetPivot - robot.PIVOT.currentPosition
        val liftError = targetLift - robot.LIFT.currentPosition

        robot.PIVOT.power = (pivotError * 0.001).coerceIn(-0.5, 0.5)
        robot.CPIVOT.power = robot.PIVOT.power
        robot.LIFT.power = (liftError * 0.001).coerceIn(-0.5, 0.5)
        robot.WRIST.position = targetWrist
    }

    override fun runOpMode() {
        val timer = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val ROBOT = Robot(hardwareMap)
        var m = 0.5

        // Initialize and reset encoders
        resetEncoders(ROBOT)

        // Initialize motors
        ROBOT.PIVOT.power = 0.0
        ROBOT.CPIVOT.power = 0.0
        ROBOT.WRIST.position = 0.0

        var currentState = ArmState.MANUAL
        var lastIntakePower = 0.0

        // Display initialization status
        telemetry.addData("Status", "Initialized")
        telemetry.addData("Constraints", if (ENABLE_CONSTRAINTS) "Enabled" else "Disabled")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("Loop Time", timer.milliseconds())
            timer.reset()

            // State machine transitions
            when {
                gamepad2.dpad_up -> currentState = ArmState.MANUAL
                gamepad2.dpad_left -> currentState = ArmState.INTAKING
                gamepad2.dpad_right -> currentState = ArmState.OUTTAKING
                gamepad2.dpad_down -> currentState = ArmState.HANGING
            }

            // Manual encoder reset during operation if needed
            if (gamepad2.share && gamepad2.options) {
                resetEncoders(ROBOT)
                telemetry.addData("Status", "Encoders Reset")
            }

            // State machine behavior
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

                    // Manual wrist control
                    if (gamepad2.right_trigger > 0.0) {
                        ROBOT.WRIST.position -= 0.01
                    } else if (gamepad2.left_trigger > 0.0) {
                        ROBOT.WRIST.position += 0.01
                    }
                }

                ArmState.INTAKING -> {
                    moveToPosition(ROBOT, INTAKE_PIVOT_POS, INTAKE_LIFT_POS, INTAKE_WRIST_POS)
                    ROBOT.INTAKE.power = 1.0
                    ROBOT.LINTAKE.power = 1.0
                }

                ArmState.OUTTAKING -> {
                    moveToPosition(ROBOT, OUTTAKE_PIVOT_POS, OUTTAKE_LIFT_POS, OUTTAKE_WRIST_POS)
                    ROBOT.INTAKE.power = -1.0
                    ROBOT.LINTAKE.power = -1.0
                }

                ArmState.HANGING -> {
                    // Retract slides first
                    if (abs(ROBOT.LIFT.currentPosition) > HANGING_THRESHOLD) {
                        ROBOT.LIFT.power = -0.5
                        ROBOT.PIVOT.power = 0.0
                        ROBOT.CPIVOT.power = 0.0
                    } else {
                        ROBOT.LIFT.power = 0.0
                        // Once slides are retracted, you can add additional hanging behavior here
                    }
                }
            }

            // Driver 1 controls remain unchanged
            when {
                gamepad1.cross -> m = 0.25
                gamepad1.circle -> m = 0.5
                gamepad1.square -> m = 0.75
                gamepad1.triangle -> m = 1.0
            }
            ROBOT.gamepadDrive(gamepad1, m)

            // Telemetry
            telemetry.addData("Current State", currentState)
            telemetry.addData("Constraints", if (ENABLE_CONSTRAINTS) "Enabled" else "Disabled")
            telemetry.addData("Pivot Position", ROBOT.PIVOT.currentPosition)
            telemetry.addData("Lift Position", ROBOT.LIFT.currentPosition)
            telemetry.addData("Wrist Position", ROBOT.WRIST.position)
            telemetry.update()
        }
    }

    private fun lerp(a: Double, b: Double, t: Double): Double {
        return a + (b - a) * t
    }
}