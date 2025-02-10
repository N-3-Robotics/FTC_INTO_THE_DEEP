package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.robot.components.DRIVETRAIN

class BOT(opMode: LinearOpMode?) {
    private val opMode: LinearOpMode? = opMode
    private val hwMap: HardwareMap? = opMode!!.hardwareMap

    private val DRIVETRAIN = DRIVETRAIN(opMode)

    init {
        DRIVETRAIN
    }
}