package org.firstinspires.ftc.teamcode.opmodes.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
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


        waitForStart()
        /* END - STARTING ROBOT */

        while (opModeIsActive()){
            telemetry.addData("Loop Time", timer.milliseconds())
            timer.reset()

            /* DRIVER 2 */


            /////////////////////////////////////


            /* ELEVATOR CONTROLS */
            if (gamepad2.left_trigger > 0) {
                ROBOT.ELEVATOR.power = -gamepad2.right_stick_y.toDouble()
            }
            else {
                ROBOT.ELEVATOR.power = 0.0
            }
            /* END ELEVATOR CONTROLS */

            /* DRIVETRAIN SPEED CONTROL */



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


            telemetry.addData("LIFT STATE", LiftState)
            telemetry.addData("Launcher Safety", Safety)

            telemetry.update()
            /* END - ACTION LOOP */
        }

        
    }

    /* END - FUNCTIONS */
}