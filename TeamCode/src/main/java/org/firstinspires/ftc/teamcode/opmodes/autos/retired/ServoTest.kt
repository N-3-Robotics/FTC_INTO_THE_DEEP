package org.firstinspires.ftc.teamcode.opmodes.autos.retired

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.Robot

@Disabled
@Config
object TestVars{

    @JvmField
    var WristLevelPos: Double = 1.0

    @JvmField
    var WristTop: Double = 0.0

    @JvmField
    var LGOpen: Double = 0.1

    @JvmField
    var LGClose: Double = 0.0

    @JvmField
    var RGOpen: Double = 0.0

    @JvmField
    var RGClose: Double = 0.1

    @JvmField
    var AUTODOWN: Int = 999

    @JvmField
    var LOCKLock: Double = 0.0

    @JvmField
    var LOCKUnlock: Double = 0.2

    @JvmField
    var LAUNCHERStaged: Double = 1.0

    @JvmField
    var LAUNCHERLaunch: Double = 0.0

    @JvmField
    var SAFETYLocked: Double = 0.4

    @JvmField
    var SAFETYUnlocked: Double = 0.0
}