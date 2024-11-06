@file:Suppress("unused", "NAME_SHADOWING")
package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.robot.AutoMode.MANUAL
import org.firstinspires.ftc.teamcode.robot.AutoMode.TURN
import org.firstinspires.ftc.teamcode.robot.AutoMode.UNKNOWN
import org.firstinspires.ftc.teamcode.robot.DriveConstants.strafeMultiplier
import org.firstinspires.ftc.teamcode.robot.QOL.Companion.ticksToInches


class Robot(hwMap: HardwareMap?) {
    var FL: DcMotorEx
    var FR: DcMotorEx
    var BL: DcMotorEx
    var BR: DcMotorEx

    var driveMotors: Array<DcMotorEx>


    var LIFT: DcMotorEx
    var PIVOT: DcMotorEx
    var PIVOTF: DcMotorEx

    var ELEVATOR: DcMotorEx

    var INTAKE: CRServo
    var LINTAKE: CRServo

    var LOCK: Servo

    var IMU: BNO055IMU

    var ODOSENSOR: SparkFunOTOS

    val trackWidth = 12.0
    val wheelBase = 9.5
    val lateralMultiplier = 1.1

    private var distanceTarget: Double = 0.0
    private var distanceError: Double = 0.0

    private var angleTarget: Double = 0.0
    private var errorAngle: Double = 0.0

    private var headingError: Double = 0.0
    private var headingTarget: Double = 0.0

    private var correction: Double = 0.0

    private var headingCorrection: Double = 0.0

    private var hasBeenRun = true

    var autoMode = UNKNOWN

    private var hardwareMap: HardwareMap? = null


    init {
        hardwareMap = hwMap


        FL = hardwareMap!!.get(DcMotorEx::class.java, "FL")
        FR = hardwareMap!!.get(DcMotorEx::class.java, "FR")
        BL = hardwareMap!!.get(DcMotorEx::class.java, "BL")
        BR = hardwareMap!!.get(DcMotorEx::class.java, "BR")

        driveMotors = arrayOf(FL, FR, BL, BR)


        LIFT = hardwareMap!!.get(DcMotorEx::class.java, "LIFT")
        LIFT.direction = DcMotorSimple.Direction.REVERSE

        PIVOT = hardwareMap!!.get(DcMotorEx::class.java, "PIVOT")
        PIVOT.direction = DcMotorSimple.Direction.REVERSE
        PIVOTF = hardwareMap!!.get(DcMotorEx::class.java, "PIVOTF")
        PIVOTF.direction = DcMotorSimple.Direction.REVERSE
        
        INTAKE = hardwareMap!!.get(CRServo::class.java, "INTAKE")
        LINTAKE = hardwareMap!!.get(CRServo::class.java, "LINTAKE")

        ELEVATOR = hardwareMap!!.get(DcMotorEx::class.java, "ELEVATOR")

        

         

        LOCK = hardwareMap!!.get(Servo::class.java, "LOCK")

        FL.direction = DcMotorSimple.Direction.REVERSE
        BL.direction = DcMotorSimple.Direction.REVERSE



        FR.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        FL.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        BR.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        BL.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        FR.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        FL.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        BR.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        BL.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER

        IMU = hardwareMap!!.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        IMU.initialize(parameters)

        ODOSENSOR = hardwareMap!!.get(SparkFunOTOS::class.java, "ODOMETER")

    }

    val currentPosition: Int
        get() {
            return (FL.currentPosition + FR.currentPosition + BL.currentPosition + BR.currentPosition) / 4
        }

    val isBusy: Boolean
        get() {
            for (motor in driveMotors) {
                if (motor.isBusy) {
                    return true
                }
            }
            return false
        }

    var zeroPowerBehavior: DcMotor.ZeroPowerBehavior
        get() {
            return FL.zeroPowerBehavior
        }
        set(value) {
            FL.zeroPowerBehavior = value
            FR.zeroPowerBehavior = value
            BL.zeroPowerBehavior = value
            BR.zeroPowerBehavior = value
        }

    var mode: DcMotor.RunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        set(value) {
            FL.mode = value
            FR.mode = value
            BL.mode = value
            BR.mode = value
            field = value
        }





    private fun tDrive(drive: Double, turn: Double){
        FL.power = drive + turn
        FR.power = drive - turn
        BL.power = drive + turn
        BR.power = drive - turn

        autoMode = MANUAL
    }

    fun RCDrive(y: Double, x: Double, rx: Double) {
        val x = x * strafeMultiplier

        //val denominator = max(abs(y) + abs(x) + abs(rx), 1.0).toDouble()

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        val leftFrontPower: Double = y + x + rx
        val rightFrontPower: Double = y - x - rx
        val leftBackPower: Double = y - x + rx
        val rightBackPower: Double = y + x - rx

        FL.power = leftFrontPower
        BL.power = leftBackPower
        FR.power = rightFrontPower
        BR.power = rightBackPower

        autoMode = MANUAL
    }

    fun forward(power: Double, timeMS: Int, opMode: LinearOpMode){
        RCDrive(power, 0.0, 0.0)
        opMode.sleep(timeMS.toLong())
        stop()
    }

    fun backward(power: Double, timeMS: Int, opMode: LinearOpMode){
        RCDrive(-power, 0.0, 0.0)
        opMode.sleep(timeMS.toLong())
        stop()
    }

    fun strafeLeft(power: Double, timeMS: Int, opMode: LinearOpMode){
        RCDrive(0.0, -power, 0.0)
        opMode.sleep(timeMS.toLong())
        stop()
    }

    fun strafeRight(power: Double, timeMS: Int, opMode: LinearOpMode){
        RCDrive(0.0, power, 0.0)
        opMode.sleep(timeMS.toLong())
        stop()
    }

//    fun FCDrive(y: Double, x: Double, turn: Double) {
//        /*val x = x * strafeMultiplier
//        val rotX = x * cos(-botHeading) - y * sin(-botHeading)
//        val rotY = y * sin(-botHeading) + x * cos(-botHeading)*/
//
//        val input = Vector2d(y, -x).rotated(-botHeading)
//
//        RCDrive(input.y, input.x, turn)
//    }

    fun gamepadDrive(controller: Gamepad, multiplier: Double) {
        RCDrive(
            -controller.left_stick_y.toDouble() * multiplier,
            controller.left_stick_x.toDouble() * multiplier,
            controller.right_stick_x.toDouble() * multiplier
        )
    }

    private fun resetHeading(){
        var globalAngle = 0.0
    }

    fun stop() {
        RCDrive(0.0, 0.0, 0.0)
        autoMode = UNKNOWN
    }

    private fun prepareMotors(){
        for (motor in driveMotors){
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
    }
    fun turnRight(angle: Int = -90){

        autoMode = TURN

        resetHeading()

        angleTarget = -angle.toDouble()

        prepareMotors()

    }

    fun turnLeft(angle: Int = 90){

        autoMode = TURN

        resetHeading()

        angleTarget = angle.toDouble()

        prepareMotors()

    }

    fun rumble(controller: Gamepad, side: Side, power: RumbleStrength, durationMS: Int = 100) {
        val pwr = power.strength
        when (side) {
            Side.LEFT -> {
                controller.rumble(pwr, 0.0, durationMS)
            }
            Side.RIGHT -> {
                controller.rumble(0.0, pwr, durationMS)
            }
            Side.BOTH -> {
                controller.rumble(pwr / 2, pwr / 2, durationMS)
            }
        }
    }

    fun getWheelPositions(): List<Double> {
        val wheelPositions: MutableList<Double> = ArrayList()
        for (motor in driveMotors) {
            wheelPositions.add(ticksToInches(motor.currentPosition))
        }
        return wheelPositions
    }

    fun getWheelVelocities(): List<Double>? {
        val wheelVelocities: MutableList<Double> = ArrayList()
        for (motor in driveMotors) {
            wheelVelocities.add(ticksToInches(motor.velocity))
        }
        return wheelVelocities
    }

    private fun setMotorPowers(v: Double, v1: Double, v2: Double, v3: Double) {
        FL.power = v
        BL.power = v1
        BR.power = v2
        FR.power = v3
    }

    fun getExternalHeadingVelocity(): Double {
        return IMU.angularVelocity.zRotationRate.toDouble()
    }



}
