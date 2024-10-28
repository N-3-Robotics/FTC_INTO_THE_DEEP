package org.firstinspires.ftc.teamcode.opmodes.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.opmodes.teleops.Lift.*


private enum class States {
    OPEN, CLOSE, UP, DOWN, LOCKED, UNLOCKED, LAUNCHED, STAGED
}
private enum class Lift {
    LOWER, READY, CLOSE, GUARANOP
}

@TeleOp(name = "TeleOp")
class TeleOP: LinearOpMode() {
    override fun runOpMode() {


        /* INITIALIZATION */
        val timer = ElapsedTime()


        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val ROBOT = Robot(hardwareMap)

        var d1Clone: Gamepad = Gamepad()
        var d2Clone: Gamepad = Gamepad()

        d1Clone.copy(gamepad1)
        d2Clone.copy(gamepad2)

        var m = 0.5


        var Safety = true

        var LiftState = READY

        var MaxExtention = 10
        var MinExtension = 1



        ROBOT.PIVOT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        ROBOT.PIVOT.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        ROBOT.LIFT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        ROBOT.LIFT.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER


        waitForStart()
        /* END - STARTING ROBOT */

        while (opModeIsActive()){
            telemetry.addData("Loop Time", timer.milliseconds())
            timer.reset()

            /* DRIVER 2 */

            val maxExtension = lerp(0.0, 1990.0, (ROBOT.PIVOT.currentPosition)/1030.0) + (1990 - 130)

            var liftPower = -gamepad2.right_stick_y.toDouble()

            // if the Lift is past the Max extension, set the power to -0.1
            if (ROBOT.LIFT.currentPosition > maxExtension) {
                liftPower = -0.5
            }



            if (-gamepad2.left_stick_y.toDouble() > 0.0) {
                ROBOT.PIVOT.power = -gamepad2.left_stick_y.toDouble() * 0.45
            }

            else if (-gamepad2.left_stick_y.toDouble() < 0.0) {
                if (ROBOT.LIFT.currentPosition > 1500) {
                    ROBOT.PIVOT.power = lerp(0.1, 0.0, gamepad2.left_stick_y.toDouble())
                }
                else {
                    ROBOT.PIVOT.power = lerp(0.1, -0.3, gamepad2.left_stick_y.toDouble())

                }
            }
            else {
                ROBOT.PIVOT.power = 0.1
            }

            // if right bumper is pressed, set the power of INTAKE to .25, else set it to 0
            if (gamepad2.right_bumper) {
                ROBOT.INTAKE.power = 1.0
            } else {
                ROBOT.INTAKE.power = 0.0
            }

            // if left bumper is pressed, set the power of INTAKE to -.25, else set it to 0
            if (gamepad2.left_bumper) {
                ROBOT.INTAKE.power = -1.0
            } else {
                ROBOT.INTAKE.power = 0.0
            }

            // if right_trigger + right_stick, set the ELEVATOR power to 1.0, else set it to 0
            // ACTUAROR CONTROLS - ELEVATOR
            if (gamepad2.right_trigger > 0) {
                ROBOT.ELEVATOR.power = -gamepad2.right_stick_y.toDouble()
            } else {
                ROBOT.ELEVATOR.power = 0.0
            }

            when {
                gamepad1.cross -> {
                    m = 0.25
                }
                gamepad1.circle -> {
                    m = 0.5
                }
                gamepad1.square -> {
                    m = 0.75
                }
                gamepad1.triangle -> {
                    m = 1.0
                }
            }
            /* END - DRIVETRAIN SPEED CONTROL */

            /* ACTION LOOP */

            d1Clone.copy(gamepad1)
            d2Clone.copy(gamepad2)

            ROBOT.gamepadDrive(gamepad1, m)

            ROBOT.LIFT.power = liftPower


            telemetry.addData("LIFT STATE", LiftState)
            telemetry.addData("Launcher Safety", Safety)

            telemetry.addData("Pivot Position", ROBOT.PIVOT.currentPosition)

            telemetry.addData("Lift Position", ROBOT.LIFT.currentPosition)

            telemetry.addData("Max Extension", maxExtension)

            telemetry.addData("pivot Power", ROBOT.PIVOT.power)

            telemetry.update()
            /* END - ACTION LOOP */
        }


        
    }

    /* END - FUNCTIONS */

    //lerp function
    private fun lerp(a: Double, b: Double, t: Double): Double {
        return a + (b - a) * t
    }
}