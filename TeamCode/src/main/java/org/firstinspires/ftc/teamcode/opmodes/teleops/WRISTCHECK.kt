package org.firstinspires.ftc.teamcode.opmodes.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robot.Robot

@TeleOp(name = "WRIST RESET")
class WRISTCHECK:  LinearOpMode() {
    override fun runOpMode() {
        val ROBOT = Robot(hardwareMap)

        waitForStart()

        ROBOT.WRIST.position = 0.5

        while (opModeIsActive()) {
            telemetry.addData("Wrist Position", ROBOT.WRIST.position)
            telemetry.update()
        }
    }
}