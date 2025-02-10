package org.firstinspires.ftc.teamcode.robot.components.subcomponents

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

class PIVOT(pivotMotor: DcMotorEx, counterPivotMotor: DcMotorEx) {
    val PIVOTM = pivotMotor
    val CPIVOTM = counterPivotMotor


    init {
        PIVOTM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        CPIVOTM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        PIVOTM.direction = DcMotorSimple.Direction.FORWARD
        CPIVOTM.direction = DcMotorSimple.Direction.REVERSE

        PIVOTM.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        CPIVOTM.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        PIVOTM.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        CPIVOTM.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    // create an action that takes the motor to a specific position
    inner class MoveToPos(tposition: Int): Action {
        private var initialized = false

        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                PIVOTM.setPower(-0.8)
                CPIVOTM.setPower(-0.8)
                initialized = true
            }

            val pos: Double = PIVOTM.getCurrentPosition().toDouble()
            packet.put("pivotPos", pos)
            if (pos < 3000.0) {
                return true
            } else {
                PIVOTM.setPower(0.0)
                return false
            }
        }
    }

    fun moveToPos(tposition: Int): Action {
        return MoveToPos(tposition)
    }

    //create an action that sets the power of the motor
    inner class SetPower(power: Double): Action {
        private val power = power

        override fun run(packet: TelemetryPacket): Boolean {
            PIVOTM.setPower(power)
            CPIVOTM.setPower(power)
            return false
        }
    }

    fun setPower(power: Double): Action {
        return SetPower(power)
    }

}