package org.firstinspires.ftc.teamcode.opmodes.autos

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TrajectoryActionFactory
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive


// import com.acmerobotics.rodrunner.drive.Drive

class Auto: LinearOpMode() {


    override fun runOpMode() {

        val initalPose = Pose2d(0.0, 0.0, 0.0)
        val theDrive = SparkFunOTOSDrive(hardwareMap, initalPose)


        waitForStart()



            val traj = theDrive.actionBuilder(initalPose)



                        .setTangent(0.0)
                        .splineToLinearHeading(Pose2d(-48.0, -48.0, 90.0), Math.PI / 2)
                        // INSERT ARM CODE HERE
                        .lineToX(5.0)
                        .turn(Math.toRadians(270.0))
                        .lineToYConstantHeading(40.0)
                        // INSERT ARM CODE HERE
                        .lineToY(0.0)
                        .turn(Math.toRadians(0.0))
                        .setTangent(0.0)
                        .splineToLinearHeading(Pose2d(-48.0, -60.0, 90.0), Math.PI / 2)
                        // INSERT ARM CODE HERE
                        .lineToX(5.0)
                        .turn(Math.toRadians(270.0))
                        .lineToYConstantHeading(40.0)


                        .build()

    }
}

// 3:14 PM 12/5/2024