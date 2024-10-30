package org.firstinspires.ftc.teamcode.opmodes.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.Robot


@Autonomous(name = "TimeAtuo", group = "Autos")
class TimeAuto: LinearOpMode() {

    override fun runOpMode() {
        val ROBOT = Robot(hwMap = hardwareMap)

        waitForStart()

        ROBOT.RCDrive(1.0, 0.0, 0.0)
        sleep(3000)
        ROBOT.stop()


    }
}