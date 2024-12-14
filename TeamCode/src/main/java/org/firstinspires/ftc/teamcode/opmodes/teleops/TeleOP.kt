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
import kotlin.math.abs


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

        val isLimited = true



        ROBOT.PIVOT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        ROBOT.PIVOT.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER


        while (ROBOT.PIVOT.currentPosition < 2167) {
            ROBOT.PIVOT.power = 0.5
        }

        ROBOT.PIVOT.power = 0.0

        ROBOT.LIFT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        ROBOT.LIFT.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        var intakePower: Double = 0.0
        var lastIntakePower = intakePower

        var liftPower: Double = 0.0
        var pivotPower: Double = 0.0

        waitForStart()
        /* END - STARTING ROBOT */

        while (opModeIsActive()){
            telemetry.addData("Loop Time", timer.milliseconds())
            timer.reset()






            /* DRIVER 2 */

            pivotPower = gamepad2.left_stick_y.toDouble()

            val pivotPOS = ROBOT.PIVOT.currentPosition

            if (pivotPOS >= 3200 && pivotPower > 0) {
                pivotPower = 0.0
            }
            else if (pivotPOS < 0 && pivotPower < 0) {
                pivotPower = 0.0
            }

            if (gamepad2.share) {
                ROBOT.PIVOT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                ROBOT.PIVOT.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }







            // 2184 is limit when all the way down.
            val maxExtension = 2184 + lerp(0.0, 3700.0-2184, (pivotPOS / 3200.0))



            liftPower = -gamepad2.right_stick_y.toDouble()

            if (liftPower == 0.0 && ROBOT.LIFT.currentPosition > 1500 && pivotPOS > 2500) {
                liftPower = 0.1
            }

            // if the Lift is past the Max extension, set the power to -0.1
            if (isLimited) {
                if (ROBOT.LIFT.currentPosition > maxExtension) {
                    liftPower = -0.5
                }
            }

            if (gamepad2.options) {
                ROBOT.LIFT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                ROBOT.LIFT.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }




            // if right bumper is pressed, set the power of INTAKE to .25, else set it to 0
            if (gamepad2.right_bumper || gamepad2.left_bumper) {
                intakePower = 1.0
            } else {
                intakePower = 0.0
            }

            // if left bumper is pressed, set the power of INTAKE to -.25, else set it to 0
            if (gamepad2.left_trigger.toDouble() > 0.0 || gamepad2.right_trigger.toDouble() > 0.0) {
                intakePower = -1.0
//                ROBOT.INTAKE.power = -1.0
//                ROBOT.LINTAKE.power = 1.0
            }



            if (intakePower != lastIntakePower) {
                ROBOT.INTAKE.power = intakePower
                ROBOT.LINTAKE.power = -intakePower
            }
            lastIntakePower = intakePower



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

            ROBOT.PIVOT.power = pivotPower


            telemetry.addData("LIFT STATE", LiftState)
            telemetry.addData("Launcher Safety", Safety)

            telemetry.addData("PIVOT POWER", pivotPower)
            telemetry.addData("Pivot Position", pivotPOS)

            telemetry.addData("Lift Position", ROBOT.LIFT.currentPosition)

            telemetry.addData("Max Extension", maxExtension)

            telemetry.addData("Intake Power", intakePower)

            telemetry.update()
            /* END - ACTION LOOP */
        }
//        // Reset the robot after the program ends
//        if (ROBOT.PIVOT.currentPosition > 0.0 || ROBOT.LIFT.currentPosition > 0.0){
//            ROBOT.PIVOT.targetPosition = 0
//            ROBOT.LIFT.targetPosition = 0
//
//            ROBOT.PIVOT.mode = DcMotor.RunMode.RUN_TO_POSITION
//            ROBOT.LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
//
//            while (ROBOT.LIFT.currentPosition > 100) {
//                sleep(500)
//            }
//        }

        
    }

    /* END - FUNCTIONS */

    //lerp function
    private fun lerp(a: Double, b: Double, t: Double): Double {
        return a + (b - a) * t
    }
}