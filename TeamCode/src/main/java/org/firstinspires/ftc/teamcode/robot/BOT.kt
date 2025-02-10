package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.robot.components.DRIVETRAIN
import org.firstinspires.ftc.teamcode.robot.components.INTAKE

class BOT(opMode: LinearOpMode?) {
    private val opMode: LinearOpMode? = opMode
    private val hwMap: HardwareMap? = opMode!!.hardwareMap

    private val DRIVETRAIN = DRIVETRAIN(opMode)
    private val INTAKE = INTAKE(opMode)

    init {
        DRIVETRAIN
        INTAKE
    }
}