package org.firstinspires.ftc.teamcode.opmodes.autos

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.forwardProfile
import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.back
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive
import org.firstinspires.ftc.teamcode.pipelines.depricated.AprilTagPipeline

class Auto: LinearOpMode() {


//        val builder = TrajectoryBuilder(drive.poseEstimate, drive.trajectoryConstraints)
//
//        builder.forward(10.0)
//        drive.followTrajectory(builder.build())

    override fun runOpMode() {

            val DRIVE = MecanumDrive(hardwareMap)
            val odo = SparkFunOTOSDrive(hardwareMap)
            val path = DRIVE.actionBuilder(Pose2d(0.0,0.0,0.0))
            val builder = TrajectoryBuilder(Pose2d(0.0, 0.0, 0.0), DRIVE.trajectoryConstraints)
            val red = false
        waitForStart()

            builder.poseEstimate = Pose2d(0.0, 0.0, 0.0)
            if (red) {
                builder.forward(10.0)
                builder.strafeRight(10.0)
            }
            else {
                builder.forward(10.0)
                builder.strafeLeft(10.0)
            }
            DRIVE.followTrajectory(path.build())


    }

    private fun TrajectoryBuilder(pose2d: Pose2d, trajectoryConstraints: Any): Any {

    return TrajectoryBuilder(pose2d, trajectoryConstraints)
    }

}

