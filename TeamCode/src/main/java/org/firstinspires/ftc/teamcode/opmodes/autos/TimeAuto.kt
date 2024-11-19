package org.firstinspires.ftc.teamcode.opmodes.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.Robot


@Autonomous(name = "Park", group = "Autos")
class TimeAuto: LinearOpMode() {

    override fun runOpMode() {
        val ROBOT = Robot(hwMap = hardwareMap)

        waitForStart()
//        ROBOT.forward(0.5,1000,this)
//        ROBOT.strafeRight(0.5, 1800, this)

    }
}