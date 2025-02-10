package org.firstinspires.ftc.teamcode.robot.components

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.robot.components.subcomponents.ACTIVE_INTAKE
import org.firstinspires.ftc.teamcode.robot.components.subcomponents.PIVOT
import org.firstinspires.ftc.teamcode.robot.components.subcomponents.SLIDES
import com.acmerobotics.roadrunner.Action


class INTAKE(opMode: LinearOpMode?) {
    private val opmode: LinearOpMode? = opMode!!
    private val hwMap = opmode!!.hardwareMap

    private val intake: ACTIVE_INTAKE
    private val slides: SLIDES
    private val pivot: PIVOT

    init {
        // Initialize the intake
        intake = ACTIVE_INTAKE(
            hwMap.get(CRServo::class.java, "LSPIN"),
            hwMap.get(CRServo::class.java, "RSPIN"),
            hwMap.get(Servo::class.java, "WRIST")
        )

        // Initialize the slides
        slides = SLIDES(hwMap.get(DcMotorEx::class.java, "SLIDES"))

        // Initialize the pivot
        pivot = PIVOT(
            hwMap.get(DcMotorEx::class.java, "PIVOTM"),
            hwMap.get(DcMotorEx::class.java, "CPIVOTM")
        )
    }

    // Intake control Action via gamepad
    inner class INTAKECONTROL : Action {
        // if right bumper is pressed, intakePower = 1.0, else intakePower = 0.0.  If left bumper is pressed, intakePower = -1.0
        val intakePower: Double = if (opmode!!.gamepad2.right_bumper) 1.0 else if (opmode!!.gamepad2.left_bumper) -1.0 else 0.0
        val slidesPower: Double = -opmode!!.gamepad2.right_stick_y.toDouble()
        val pivotPower: Double = opmode!!.gamepad2.left_stick_y.toDouble()
        // if right trigger is pressed, wrist power is 1.0, else if left trigger is pressed, wrist power is -1.0, else wrist power is 0.0
        val wristPower: Double = if (opmode!!.gamepad2.right_trigger.toDouble() > 0.0) 0.1 else if (opmode!!.gamepad2.left_trigger.toDouble() > 0.0) -0.1 else 0.0

        override fun run(p: TelemetryPacket): Boolean {
            intake.spin(intakePower)
            slides.setPower(slidesPower)
            pivot.setPower(pivotPower)
            intake.moveWrist(wristPower)
            return false
        }
    }

}