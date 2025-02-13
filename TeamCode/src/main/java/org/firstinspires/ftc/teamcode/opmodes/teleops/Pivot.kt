package org.firstinspires.ftc.teamcode.opmodes.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.robot.Robot

@TeleOp(name="Pivot Movement")
class Pivot: LinearOpMode() {
    override fun runOpMode() {
        val ROBOT = Robot(hardwareMap)

        waitForStart()

        ROBOT.PIVOT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        ROBOT.PIVOT.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        while (opModeIsActive()) {
            ROBOT.PIVOT.power = gamepad2.left_stick_y.toDouble()
            ROBOT.CPIVOT.power = gamepad2.left_stick_y.toDouble()

            telemetry.addData("Position", ROBOT.PIVOT.currentPosition)
            telemetry.addData("WRIST POS", ROBOT.WRIST.position)
            telemetry.update()
        }

    }
}