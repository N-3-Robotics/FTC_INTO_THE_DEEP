package org.firstinspires.ftc.teamcode.robot.components.subcomponents

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.Servo
import com.acmerobotics.roadrunner.Action

class ACTIVE_INTAKE(leftSpinner: CRServo, rightSpinner: CRServo, wrist: Servo) {
    private val LSPIN = leftSpinner
    private val RSPIN = rightSpinner
    private val WRIST = wrist

    private var lastSpinPower = 0.0
    private var lastWristPower = 0.0

    inner class SPIN(val power: Double) : Action {
        override fun run(p: TelemetryPacket): Boolean {
            if (power != lastSpinPower) {
                LSPIN.power = power
                RSPIN.power = -power
                lastSpinPower = power
            }
            return false
        }
    }

    inner class WRIST_GOTOPOS(val position: Double) : Action {
        override fun run(p: TelemetryPacket): Boolean {
            WRIST.position = position
            return false
        }
    }

    fun spin(power: Double): Action {
        return SPIN(power)
    }

    fun setWristPos(position: Double): Action {
        return WRIST_GOTOPOS(position)
    }

    // Action to allow the wrist to move like a motor
    inner class MoveWrist(val power: Double) : Action {
        override fun run(p: TelemetryPacket): Boolean {
            if (power != lastWristPower) {
                WRIST.position += power
                lastWristPower = power
            }
            return false
        }
    }

    fun moveWrist(power: Double): Action {
        return MoveWrist(power)
    }
}