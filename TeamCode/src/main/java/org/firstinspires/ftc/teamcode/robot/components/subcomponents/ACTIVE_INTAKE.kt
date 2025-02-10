package org.firstinspires.ftc.teamcode.robot.components.subcomponents

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.Servo

class ACTIVE_INTAKE(leftSpinner: CRServo, rightSpinner: CRServo, wrist: Servo) {
    private val LSPIN = leftSpinner
    private val RSPIN = rightSpinner
    private val WRIST = wrist

    private var lastSpinPower = 0.0
    private var lastWristPower = 0.0


    fun spin(power: Double) {
        if (power != lastSpinPower) {
            LSPIN.power = power
            RSPIN.power = -power
            lastSpinPower = power
        }
    }

    fun wristGoToPos(pos: Double) {
        WRIST.position = pos
    }


    // Action to allow the wrist to move like a motor
    fun setWristPower(power: Double) {
        if (power != lastWristPower) {
            WRIST.position += power
            lastWristPower = power
        }
    }


}