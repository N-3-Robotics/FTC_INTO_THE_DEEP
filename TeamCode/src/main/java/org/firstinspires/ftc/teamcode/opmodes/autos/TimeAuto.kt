package org.firstinspires.ftc.teamcode.opmodes.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.robot.Robot




@Autonomous(name = "Park", group = "Autos")
class TimeAuto: LinearOpMode() {

    override fun runOpMode() {
        val ROBOT = Robot(hwMap = hardwareMap)

        ROBOT.PIVOT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        ROBOT.PIVOT.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER


        while (ROBOT.PIVOT.currentPosition < 2167) {
            ROBOT.PIVOT.power = 0.5
        }
        ROBOT.PIVOT.power = 0.0

        waitForStart()
        ROBOT.forward(0.5,1000,this)
        ROBOT.strafeRight(0.5, 1800, this)

    }
}