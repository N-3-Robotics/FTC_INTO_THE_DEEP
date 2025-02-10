package org.firstinspires.ftc.teamcode.robot.components.subcomponents


import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs

class SLIDES(slideMotor: DcMotorEx) {
    val SLIDEM = slideMotor

    init {
        SLIDEM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        SLIDEM.setDirection(DcMotorSimple.Direction.FORWARD)
    }

    //ACTION to set the power of the slides
    inner class SETPOWER(power: Double) : Action {
        val power = power

        override fun run(p: TelemetryPacket): Boolean {
            SLIDEM.power = power
            return false
        }
    }

    fun setPower(power: Double): Action {
        return SETPOWER(power)
    }

    //ACTION to set the position of the slides
    inner class GOTOPOS(tPos: Int): Action {
        private var initialized = false
        private val tPos = tPos.toDouble()

        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                SLIDEM.setPower(0.8)
                initialized = true
            }

            val pos: Double = SLIDEM.getCurrentPosition().toDouble()
            packet.put("liftPos", pos)
            if (abs(tPos - pos) > 10) {
                return true
            } else {
                SLIDEM.setPower(0.0)
                return false
            }
        }
    }

    fun goToPos(tPos: Int): Action {
        return GOTOPOS(tPos)
    }
}