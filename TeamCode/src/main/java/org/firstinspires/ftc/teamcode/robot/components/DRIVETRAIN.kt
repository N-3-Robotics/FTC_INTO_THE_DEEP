package org.firstinspires.ftc.teamcode.robot.components

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.robot.DriveConstants.strafeMultiplier

class DRIVETRAIN(opmode: LinearOpMode?) {
    private val opmode: LinearOpMode? = opmode
    private val hwMap: HardwareMap? = opmode!!.hardwareMap

    private val drive: MecanumDrive
    var initialPose: Pose2d = Pose2d(24.0, -(60 + ((24 - 17).toDouble() / 2)), Math.toRadians(90.0))


    private val multiplier: Double = 0.5


    private var controllable: Boolean = true



    init {
        drive = MecanumDrive(hwMap, initialPose)
    }

    inner class GPDRIVE(x: Double, y: Double, rx: Double) : Action {

        val x = x * strafeMultiplier
        val y = y
        val rx = rx


        override fun run(p: TelemetryPacket): Boolean {

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            val leftFrontPower: Double = y + x + rx
            val rightFrontPower: Double = y - x - rx
            val leftBackPower: Double = y - x + rx
            val rightBackPower: Double = y + x - rx

            drive.leftFront.power = leftFrontPower
            drive.leftBack.power = leftBackPower
            drive.rightFront.power = rightFrontPower
            drive.rightBack.power = rightBackPower

            drive.updatePoseEstimate()
            return false
        }
    }

    fun gamepadDrive(): Action {
        return GPDRIVE(opmode!!.gamepad1.left_stick_x.toDouble(), -opmode!!.gamepad1.left_stick_y.toDouble(), opmode!!.gamepad1.right_stick_x.toDouble())
    }

    //create an action that takes the robot to a specific position



}
       