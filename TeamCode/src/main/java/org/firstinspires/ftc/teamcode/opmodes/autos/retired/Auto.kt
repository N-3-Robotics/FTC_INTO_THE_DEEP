package org.firstinspires.ftc.teamcode.opmodes.autos.retired

//import org.firstinspires.ftc.teamcode.robot.Robot
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.opmodes.autos.retired.Alliance.*
import org.firstinspires.ftc.teamcode.opmodes.autos.retired.Position.*
import org.firstinspires.ftc.teamcode.robot.Camera
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.util.LinkedList
import java.util.Queue

enum class Alliance {
    RED, BLUE, UNKNOWN
}
enum class Position {
    CLOSE, FAR
}

@Disabled
// ALL PATHS ARE BASED ON THIS STARTING POSE
@Autonomous(name = "AUTO")
class AutoBlueFar: LinearOpMode() {


    override fun runOpMode() {

        val DIS = hardwareMap!!.get(Rev2mDistanceSensor::class.java, "DIS")


        val CAMERA = Camera(hardwareMap, telemetry);

        sleep(3000)

        telemetry.addLine()

        val team = CAMERA.alli.getAlliance()

        when (team) {
            "RED" -> {
                CAMERA.COLOR = RED
            }

            "BLUE" -> {
                CAMERA.COLOR = BLUE
            }

            else -> CAMERA.COLOR = BLUE
        }

        CAMERA.setContour()

        val LIFT_HEIGHT = 1600

        val RETURN_POS = 62.0

        val ALLIANCE = CAMERA.COLOR



        val POSITION: Position = if (DIS.getDistance(DistanceUnit.INCH) < 6) {
            if (ALLIANCE == BLUE) {
                CLOSE
            }
            else {
                FAR
            }
        }
        else {
            if (ALLIANCE == BLUE) {
                FAR
            }
            else {
                CLOSE
            }
        }


        val color = if (ALLIANCE == BLUE) {
            1
        } else {
            -1
        }

        val DRIVE = SampleMecanumDrive(hardwareMap);

        var StartPose: Pose2d
        var pathToFollow: Queue<TrajectorySequence> = LinkedList()

        var LIFT: DcMotorEx
        var WRIST: Servo
        var LG: Servo
        var RG: Servo

        var Mult: Int = 2

        LIFT = hardwareMap!!.get(DcMotorEx::class.java, "LIFT")
        LIFT.direction = DcMotorSimple.Direction.REVERSE
        LIFT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
        LIFT.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        WRIST = hardwareMap!!.get(Servo::class.java, "WRIST")
        LG = hardwareMap!!.get(Servo::class.java, "LG")
        RG = hardwareMap!!.get(Servo::class.java, "RG")

        LG.position = 0.0
        RG.position = 0.1
        WRIST.position = TestVars.WristLevelPos



        /* CONFIGURE STARTING POSE*/
        StartPose = if (ALLIANCE == BLUE && POSITION == CLOSE) {
            Pose2d(12.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
        } else if (ALLIANCE == BLUE && POSITION == FAR) {
            Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
        } else if (ALLIANCE == RED && POSITION == FAR) {
            Pose2d(-35.0, -72.0 + 17 / 2, Math.toRadians(90.0))
        } else if (ALLIANCE == RED && POSITION == CLOSE) {
            Pose2d(12.0, -72.0 + 17 / 2, Math.toRadians(90.0))
        }
        else {
            Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
        }
        DRIVE.poseEstimate = StartPose
        DRIVE.updatePoseEstimate()
        /* END CONFIGURE STARTING POSE */


        /* SEQUENCES */
        val start = DRIVE.trajectorySequenceBuilder(StartPose)
                .forward(38 - (17.0 / 2.0))
                .build()

        val left = DRIVE.trajectorySequenceBuilder(start.end())
                .turn(Math.toRadians(90.0))
                .forward(6.0)
                .addDisplacementMarker {
                    RG.position = 0.0
                }
                .back(8.0)
                .lineToLinearHeading(Pose2d(StartPose.x, StartPose.y - (color*(42-(17.0/2.0))), Math.toRadians(color*-90.0)))
                .build()

        val right = DRIVE.trajectorySequenceBuilder(start.end())
                .turn(Math.toRadians(-90.0))
                .forward(6.0)
                .addDisplacementMarker {
                    RG.position = 0.0
                }
                .back(8.0)
                .lineToLinearHeading(Pose2d(StartPose.x, StartPose.y - (color*(42-(17.0/2.0))), Math.toRadians(color*-90.0)))
                .build()

        val center = DRIVE.trajectorySequenceBuilder(start.end())
                .forward(3.0)
                .waitSeconds(1.0)
                .addDisplacementMarker{
                    RG.position = 0.0
                }
                .waitSeconds(1.0)
                .build()




        /* END SEQUENCES */



        while (!isStarted) {
            telemetry.addData("ALLIANCE", ALLIANCE)
            telemetry.addData("SIDE", POSITION)
            telemetry.addData("Cube Location", CAMERA.cubeLocation)
            telemetry.update()
        }

        waitForStart();

        pathToFollow.add(start)

        var after: TrajectorySequence = center
        when (CAMERA.cubeLocation) {
            "CENTER" -> {
                after = center
                pathToFollow.add(center)
                Mult = 2
            }
            "LEFT" -> {
                after = left
                pathToFollow.add(left)
                Mult = if (ALLIANCE == BLUE) {
                    1
                }
                else {
                    3
                }
            }
            "RIGHT" -> {
                after = right
                pathToFollow.add(right)
                Mult = if (ALLIANCE == BLUE) {
                    3
                }
                else {
                    1
                }
            }
        }

        val returnToStartPos = DRIVE.trajectorySequenceBuilder(after.end())
                .lineToLinearHeading(Pose2d(StartPose.x, color*RETURN_POS, StartPose.heading))
                .build()

        pathToFollow.add(returnToStartPos)
        val toBoard = if (ALLIANCE == BLUE) {
            DRIVE.trajectorySequenceBuilder(returnToStartPos.end())
                    .lineTo(Vector2d(38.0, color*RETURN_POS))
                    .addDisplacementMarker {
                        RG.position = 0.1
                        LIFT.targetPosition = LIFT_HEIGHT
                        LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
                        LIFT.velocity = 800.0
                        telemetry.update()
                    }
                    .lineToLinearHeading(Pose2d(38.0, color * RETURN_POS - (color * Mult * 6) - (color * 12.0), Math.toRadians(0.0)))
                    .addDisplacementMarker{
                        WRIST.position = TestVars.WristTop
                    }
                    .forward(16.0)
                    .build()
        } else {

            DRIVE.trajectorySequenceBuilder(returnToStartPos.end())
                    .lineTo(Vector2d(38.0, color*RETURN_POS))
                    .addDisplacementMarker {
                        RG.position = 0.1
                        LIFT.targetPosition = LIFT_HEIGHT
                        LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
                        LIFT.velocity = 800.0
                    }
                    .lineToLinearHeading(Pose2d(38.0, color * RETURN_POS - (color * Mult * 6) - (color * 14.0), Math.toRadians(0.0)))
                    .addDisplacementMarker{
                        WRIST.position = TestVars.WristTop
                    }
                    .forward(16.0)
                    .build()
        }
        pathToFollow.add(toBoard)



        telemetry.addData("Status", "Running");
        telemetry.update();


        while (!pathToFollow.isEmpty()) {
            DRIVE.followTrajectorySequence(pathToFollow.poll())
        }

        val park = DRIVE.trajectorySequenceBuilder(toBoard.end())
                .addDisplacementMarker(){
                    LG.position = 0.1
                }
                .back(14.0)
                .UNSTABLE_addTemporalMarkerOffset(1.0) {
                    LIFT.velocity = 0.0
                    LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
                    LG.position = 0.0
                    WRIST.position = TestVars.WristLevelPos
                    LIFT.targetPosition = 0
                    LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
                    LIFT.velocity = 800.0
                }
                .lineTo(Vector2d(42.0, color*RETURN_POS))
                .UNSTABLE_addTemporalMarkerOffset(1.0){
                    LIFT.velocity = 0.0
                    LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
                }
                .forward(24.0)
                .build()

        DRIVE.followTrajectorySequence(park)

        stop()
    }
}

//@Autonomous(name = "RED FAR")
//class AutoRedFar: LinearOpMode() {


            /* CONFIGURE STARTING POSE*/
//    @Autonomous(name = "BLUE CLOSE")
//    class AutoBlueClose: LinearOpMode() {
//
//        override fun runOpMode() {
//
//            val LIFT_HEIGHT = 1600
//
//            val RETURN_POS = 62.0
//
//            val ALLIANCE = BLUE
//            val POSITION = CLOSE
//
//            val color = if (ALLIANCE == BLUE) {
//                1
//            } else {
//                -1
//            }
//
//            val CAMERA = Camera(hardwareMap, telemetry);
//
//            val DRIVE = SampleMecanumDrive(hardwareMap);
//            var StartPose: Pose2d
//            var pathToFollow: Queue<TrajectorySequence> = LinkedList()
//
//            var LIFT: DcMotorEx
//            var WRIST: Servo
//            var LG: Servo
//            var RG: Servo
//
//            var Mult: Int = 2
//
//            LIFT = hardwareMap!!.get(DcMotorEx::class.java, "LIFT")
//            LIFT.direction = DcMotorSimple.Direction.REVERSE
//            LIFT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
//            LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
//            LIFT.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//
//            WRIST = hardwareMap!!.get(Servo::class.java, "WRIST")
//            LG = hardwareMap!!.get(Servo::class.java, "LG")
//            RG = hardwareMap!!.get(Servo::class.java, "RG")
//
//            LG.position = 0.0
//            RG.position = 0.1
//            WRIST.position = TestVars.WristLevelPos
//
//            StartPose = if (ALLIANCE == BLUE && POSITION == CLOSE) {
//                Pose2d(12.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
//            } else if (ALLIANCE == BLUE && POSITION == FAR) {
//                Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
//            } else if (ALLIANCE == RED && POSITION == FAR) {
//                Pose2d(-35.0, -72.0 + 17 / 2, Math.toRadians(90.0))
//            } else if (ALLIANCE == RED && POSITION == CLOSE) {
//                Pose2d(12.0, -72.0 + 17 / 2, Math.toRadians(90.0))
//            }
//            else {
//                Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
//            }
//            DRIVE.poseEstimate = StartPose
//            DRIVE.updatePoseEstimate()
//            /* END CONFIGURE STARTING POSE */
//
//
//            /* SEQUENCES */
//            val start = DRIVE.trajectorySequenceBuilder(StartPose)
//                    .forward(38 - (17.0 / 2.0))
//                    .build()
//
//            val left = DRIVE.trajectorySequenceBuilder(start.end())
//                    .turn(Math.toRadians(90.0))
//                    .forward(6.0)
//                    .addDisplacementMarker {
//                        RG.position = 0.0
//                    }
//                    .back(8.0)
//                    .lineToLinearHeading(Pose2d(StartPose.x, StartPose.y - (color*(42-(17.0/2.0))), Math.toRadians(color*-90.0)))
//                    .build()
//
//            val right = DRIVE.trajectorySequenceBuilder(start.end())
//                    .turn(Math.toRadians(-90.0))
//                    .forward(6.0)
//                    .addDisplacementMarker {
//                        RG.position = 0.0
//                    }
//                    .back(8.0)
//                    .lineToLinearHeading(Pose2d(StartPose.x, StartPose.y - (color*(42-(17.0/2.0))), Math.toRadians(color*-90.0)))
//                    .build()
//
//            val center = DRIVE.trajectorySequenceBuilder(start.end())
//                    .forward(3.0)
//                    .waitSeconds(1.0)
//                    .addDisplacementMarker{
//                        RG.position = 0.0
//                    }
//                    .waitSeconds(1.0)
//                    .build()
//
//
//
//
//            /* END SEQUENCES */
//
//            while (!isStarted) {
//                telemetry.addData("Cube Location", CAMERA.cubeLocation)
//                telemetry.update()
//            }
//
//            waitForStart();
//
//            pathToFollow.add(start)
//
//            var after: TrajectorySequence = center
//            when (CAMERA.cubeLocation) {
//                "CENTER" -> {
//                    after = center
//                    pathToFollow.add(center)
//                    Mult = 2
//                }
//                "LEFT" -> {
//                    after = left
//                    pathToFollow.add(left)
//                    Mult = 1
//                }
//                "RIGHT" -> {
//                    after = right
//                    pathToFollow.add(right)
//                    Mult = 3
//                }
//            }
//
//            val returnToStartPos = DRIVE.trajectorySequenceBuilder(center.end())
//                    .lineToLinearHeading(Pose2d(StartPose.x, color*RETURN_POS, StartPose.heading))
//                    .build()
//
//            pathToFollow.add(returnToStartPos)
//            val toBoard = if (POSITION == CLOSE) {
//                DRIVE.trajectorySequenceBuilder(returnToStartPos.end())
//                        .lineTo(Vector2d(38.0, color*RETURN_POS))
//                        .addDisplacementMarker {
//                            RG.position = 0.1
//                            LIFT.targetPosition = LIFT_HEIGHT
//                            LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
//                            LIFT.velocity = 800.0
//                            telemetry.update()
//                        }
//                        .lineToLinearHeading(Pose2d(38.0, color * RETURN_POS - (color * Mult * 6) - (color * 14.0), Math.toRadians(0.0)))
//                        .addDisplacementMarker{
//                            WRIST.position = TestVars.WristTop
//                        }
//                        .forward(16.0)
//                        .build()
//            } else {
//
//                DRIVE.trajectorySequenceBuilder(returnToStartPos.end())
//                        .lineTo(Vector2d(38.0, color*RETURN_POS))
//                        .addDisplacementMarker {
//                            RG.position = 0.1
//                            LIFT.targetPosition = LIFT_HEIGHT
//                            LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
//                            LIFT.velocity = 800.0
//                        }
//                        .lineToLinearHeading(Pose2d(38.0, color * RETURN_POS -(color * Mult * 6) - (color * 14.0), Math.toRadians(0.0)))
//                        .addDisplacementMarker{
//                            WRIST.position = TestVars.WristTop
//                        }
//                        .forward(16.0)
//                        .build()
//            }
//            pathToFollow.add(toBoard)
//
//
//
//            telemetry.addData("Status", "Running");
//            telemetry.update();
//
//
//            while (!pathToFollow.isEmpty()) {
//                DRIVE.followTrajectorySequence(pathToFollow.poll())
//            }
//
//            val park = DRIVE.trajectorySequenceBuilder(toBoard.end())
//                    .addDisplacementMarker(){
//                        LG.position = 0.1
//                    }
//                    .back(14.0)
//                    .UNSTABLE_addTemporalMarkerOffset(1.0) {
//                        LIFT.velocity = 0.0
//                        LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
//                        LG.position = 0.0
//                        WRIST.position = TestVars.WristLevelPos
//                        LIFT.targetPosition = 0
//                        LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
//                        LIFT.velocity = 800.0
//                    }
//                    .lineTo(Vector2d(42.0, color*RETURN_POS))
//                    .UNSTABLE_addTemporalMarkerOffset(1.0){
//                        LIFT.velocity = 0.0
//                        LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
//                    }
//                    .forward(24.0)
//                    .build()
//
//            DRIVE.followTrajectorySequence(park)
//
//            stop()
//        }
//    }
//
//    override fun runOpMode() {
//
//        val LIFT_HEIGHT = 1600
//
//        val RETURN_POS = 60.0
//
//        val ALLIANCE = RED
//        val POSITION = FAR
//
//        val color = if (ALLIANCE == BLUE) {
//            1
//        } else {
//            -1
//        }
//
//        val CAMERA = Camera(hardwareMap, telemetry);
//
//        val DRIVE = SampleMecanumDrive(hardwareMap);
//        var StartPose: Pose2d
//        var pathToFollow: Queue<TrajectorySequence> = LinkedList()
//
//        var LIFT: DcMotorEx
//        var WRIST: Servo
//        var LG: Servo
//        var RG: Servo
//
//        var Mult: Int = 2
//
//        LIFT = hardwareMap!!.get(DcMotorEx::class.java, "LIFT")
//        LIFT.direction = DcMotorSimple.Direction.REVERSE
//        LIFT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
//        LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
//        LIFT.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//
//        WRIST = hardwareMap!!.get(Servo::class.java, "WRIST")
//        LG = hardwareMap!!.get(Servo::class.java, "LG")
//        RG = hardwareMap!!.get(Servo::class.java, "RG")
//
//        LG.position = 0.0
//        RG.position = 0.1
//        WRIST.position = TestVars.WristLevelPos
//
//
//
//        /* CONFIGURE STARTING POSE*/
//        StartPose = if (ALLIANCE == BLUE && POSITION == CLOSE) {
//            Pose2d(12.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
//        } else if (ALLIANCE == BLUE && POSITION == FAR) {
//            Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
//        } else if (ALLIANCE == RED && POSITION == FAR) {
//            Pose2d(-35.0, -72.0 + 17 / 2, Math.toRadians(90.0))
//        } else if (ALLIANCE == RED && POSITION == CLOSE) {
//            Pose2d(12.0, -72.0 + 17 / 2, Math.toRadians(90.0))
//        }
//        else {
//            Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
//        }
//        DRIVE.poseEstimate = StartPose
//        DRIVE.updatePoseEstimate()
//        /* END CONFIGURE STARTING POSE */
//
//
//        /* SEQUENCES */
//        val start = DRIVE.trajectorySequenceBuilder(StartPose)
//                .forward(38 - (17.0 / 2.0))
//                .build()
//
//        val left = DRIVE.trajectorySequenceBuilder(start.end())
//                .turn(Math.toRadians(90.0))
//                .forward(6.0)
//                .addDisplacementMarker {
//                    RG.position = 0.0
//                }
//                .back(8.0)
//                .lineToLinearHeading(Pose2d(StartPose.x, StartPose.y - (color*(42-(17.0/2.0))), Math.toRadians(color*-90.0)))
//                .build()
//
//        val right = DRIVE.trajectorySequenceBuilder(start.end())
//                .turn(Math.toRadians(-90.0))
//                .forward(6.0)
//                .addDisplacementMarker {
//                    RG.position = 0.0
//                }
//                .back(8.0)
//                .lineToLinearHeading(Pose2d(StartPose.x, StartPose.y - (color*(42-(17.0/2.0))), Math.toRadians(color*-90.0)))
//                .build()
//
//        val center = DRIVE.trajectorySequenceBuilder(start.end())
//                .forward(3.0)
//                .waitSeconds(1.0)
//                .addDisplacementMarker{
//                    RG.position = 0.0
//                }
//                .waitSeconds(1.0)
//                .build()
//
//
//
//
//        /* END SEQUENCES */
//
//        while (!isStarted) {
//            telemetry.addData("Cube Location", CAMERA.cubeLocation)
//            telemetry.update()
//        }
//
//        waitForStart();
//
//        pathToFollow.add(start)
//
//        var after: TrajectorySequence = center
//        when (CAMERA.cubeLocation) {
//            "CENTER" -> {
//                after = center
//                pathToFollow.add(center)
//                Mult = 2
//            }
//            "LEFT" -> {
//                after = left
//                pathToFollow.add(left)
//                Mult = 3
//            }
//            "RIGHT" -> {
//                after = right
//                pathToFollow.add(right)
//                Mult = 1
//            }
//        }
//
//        val returnToStartPos = DRIVE.trajectorySequenceBuilder(center.end())
//                .lineToLinearHeading(Pose2d(StartPose.x, color*RETURN_POS, StartPose.heading))
//                .build()
//
//        pathToFollow.add(returnToStartPos)
//        val toBoard = if (POSITION == CLOSE) {
//            DRIVE.trajectorySequenceBuilder(returnToStartPos.end())
//                    .lineTo(Vector2d(40.0, color*RETURN_POS))
//                    .addDisplacementMarker {
//                        RG.position = 0.1
//                        LIFT.targetPosition = LIFT_HEIGHT
//                        LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
//                        LIFT.velocity = 800.0
//                        telemetry.update()
//                    }
//                    .lineToLinearHeading(Pose2d(36.0, color * RETURN_POS - (color * Mult * 6) - (color * 14.0), Math.toRadians(0.0)))
//                    .addDisplacementMarker{
//                        WRIST.position = TestVars.WristTop
//                    }
//                    .forward(20.0)
//                    .build()
//        } else {
//
//            DRIVE.trajectorySequenceBuilder(returnToStartPos.end())
//                    .lineTo(Vector2d(40.0, color*RETURN_POS))
//                    .addDisplacementMarker {
//                        RG.position = 0.1
//                        LIFT.targetPosition = LIFT_HEIGHT
//                        LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
//                        LIFT.velocity = 800.0
//                    }
//                    .lineToLinearHeading(Pose2d(40.0, color * RETURN_POS -(color * Mult * 6) - (color * 14.0), Math.toRadians(0.0)))
//                    .addDisplacementMarker{
//                        WRIST.position = TestVars.WristTop
//                    }
//                    .forward(16.0)
//                    .build()
//        }
//        pathToFollow.add(toBoard)
//
//
//
//        telemetry.addData("Status", "Running");
//        telemetry.update();
//
//
//        while (!pathToFollow.isEmpty()) {
//            DRIVE.followTrajectorySequence(pathToFollow.poll())
//        }
//
//        val park = DRIVE.trajectorySequenceBuilder(toBoard.end())
//                .addDisplacementMarker(){
//                    LG.position = 0.1
//                }
//                .back(14.0)
//                .UNSTABLE_addTemporalMarkerOffset(1.0) {
//                    LIFT.velocity = 0.0
//                    LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
//                    LG.position = 0.0
//                    WRIST.position = TestVars.WristLevelPos
//                    LIFT.targetPosition = 0
//                    LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
//                    LIFT.velocity = 800.0
//                }
//                .lineTo(Vector2d(42.0, color*RETURN_POS))
//                .UNSTABLE_addTemporalMarkerOffset(1.0){
//                    LIFT.velocity = 0.0
//                    LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
//                }
//                .forward(24.0)
//                .build()
//
//        DRIVE.followTrajectorySequence(park)
//
//        stop()
//    }
//}
//
//@Autonomous(name = "RED CLOSE")
//class AutoRedClose: LinearOpMode() {
//
//    override fun runOpMode() {
//
//        val LIFT_HEIGHT = 1600
//
//        val RETURN_POS = 62.0
//
//        val ALLIANCE = RED
//        val POSITION = CLOSE
//
//        val color = if (ALLIANCE == BLUE) {
//            1
//        } else {
//            -1
//        }
//
//        val CAMERA = Camera(hardwareMap, telemetry);
//
//        val DRIVE = SampleMecanumDrive(hardwareMap);
//        var StartPose: Pose2d
//        var pathToFollow: Queue<TrajectorySequence> = LinkedList()
//
//        var LIFT: DcMotorEx
//        var WRIST: Servo
//        var LG: Servo
//        var RG: Servo
//
//        var Mult: Int = 2
//
//        LIFT = hardwareMap!!.get(DcMotorEx::class.java, "LIFT")
//        LIFT.direction = DcMotorSimple.Direction.REVERSE
//        LIFT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
//        LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
//        LIFT.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
//
//        WRIST = hardwareMap!!.get(Servo::class.java, "WRIST")
//        LG = hardwareMap!!.get(Servo::class.java, "LG")
//        RG = hardwareMap!!.get(Servo::class.java, "RG")
//
//        LG.position = 0.0
//        RG.position = 0.1
//        WRIST.position = TestVars.WristLevelPos
//
//
//
//        /* CONFIGURE STARTING POSE*/
//        StartPose = if (ALLIANCE == BLUE && POSITION == CLOSE) {
//            Pose2d(12.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
//        } else if (ALLIANCE == BLUE && POSITION == FAR) {
//            Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
//        } else if (ALLIANCE == RED && POSITION == FAR) {
//            Pose2d(-35.0, -72.0 + 17 / 2, Math.toRadians(90.0))
//        } else if (ALLIANCE == RED && POSITION == CLOSE) {
//            Pose2d(12.0, -72.0 + 17 / 2, Math.toRadians(90.0))
//        }
//        else {
//            Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
//        }
//        DRIVE.poseEstimate = StartPose
//        DRIVE.updatePoseEstimate()
//        /* END CONFIGURE STARTING POSE */
//
//
//        /* SEQUENCES */
//        val start = DRIVE.trajectorySequenceBuilder(StartPose)
//                .forward(38 - (17.0 / 2.0))
//                .build()
//
//        val left = DRIVE.trajectorySequenceBuilder(start.end())
//                .turn(Math.toRadians(90.0))
//                .forward(6.0)
//                .addDisplacementMarker {
//                    RG.position = 0.0
//                }
//                .back(8.0)
//                .lineToLinearHeading(Pose2d(StartPose.x, StartPose.y - (color*(42-(17.0/2.0))), Math.toRadians(color*-90.0)))
//                .build()
//
//        val right = DRIVE.trajectorySequenceBuilder(start.end())
//                .turn(Math.toRadians(-90.0))
//                .forward(6.0)
//                .addDisplacementMarker {
//                    RG.position = 0.0
//                }
//                .back(8.0)
//                .lineToLinearHeading(Pose2d(StartPose.x, StartPose.y - (color*(42-(17.0/2.0))), Math.toRadians(color*-90.0)))
//                .build()
//
//        val center = DRIVE.trajectorySequenceBuilder(start.end())
//                .forward(3.0)
//                .waitSeconds(1.0)
//                .addDisplacementMarker{
//                    RG.position = 0.0
//                }
//                .waitSeconds(1.0)
//                .build()
//
//
//
//
//        /* END SEQUENCES */
//
//        while (!isStarted) {
//            telemetry.addData("Cube Location", CAMERA.cubeLocation)
//            telemetry.update()
//        }
//
//        waitForStart();
//
//        pathToFollow.add(start)
//
//        var after: TrajectorySequence = center
//        when (CAMERA.cubeLocation) {
//            "CENTER" -> {
//                after = center
//                pathToFollow.add(center)
//                Mult = 2
//            }
//            "LEFT" -> {
//                after = left
//                pathToFollow.add(left)
//                Mult = 3
//            }
//            "RIGHT" -> {
//                after = right
//                pathToFollow.add(right)
//                Mult = 1
//            }
//        }
//
//        val returnToStartPos = DRIVE.trajectorySequenceBuilder(center.end())
//                .lineToLinearHeading(Pose2d(StartPose.x, color*RETURN_POS, StartPose.heading))
//                .build()
//
//        pathToFollow.add(returnToStartPos)
//        val toBoard = if (POSITION == CLOSE) {
//            DRIVE.trajectorySequenceBuilder(returnToStartPos.end())
//                    .lineTo(Vector2d(40.0, color*RETURN_POS))
//                    .addDisplacementMarker {
//                        RG.position = 0.1
//                        LIFT.targetPosition = LIFT_HEIGHT
//                        LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
//                        LIFT.velocity = 800.0
//                        telemetry.update()
//                    }
//                    .lineToLinearHeading(Pose2d(40.0, color * RETURN_POS - (color * Mult * 6) - (color * 14.0), Math.toRadians(0.0)))
//                    .addDisplacementMarker{
//                        WRIST.position = TestVars.WristTop
//                    }
//                    .forward(16.0)
//                    .build()
//        } else {
//            /* RESIDUAL.  DISREGARD THIS SECTION */
//            DRIVE.trajectorySequenceBuilder(returnToStartPos.end())
//                    .lineTo(Vector2d(40.0, color*RETURN_POS))
//                    .addDisplacementMarker {
//                        RG.position = 0.1
//                        LIFT.targetPosition = LIFT_HEIGHT
//                        LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
//                        LIFT.velocity = 800.0
//                    }
//                    .lineToLinearHeading(Pose2d(36.0, color * RETURN_POS - (color * Mult * 6) - (color * 14.0), Math.toRadians(0.0)))
//                    .addDisplacementMarker{
//                        WRIST.position = TestVars.WristTop
//                    }
//                    .forward(16.0)
//                    .build()
//            /*---------------------------------*/
//        }
//        pathToFollow.add(toBoard)
//
//
//
//        telemetry.addData("Status", "Running");
//        telemetry.update();
//
//
//        while (!pathToFollow.isEmpty()) {
//            DRIVE.followTrajectorySequence(pathToFollow.poll())
//        }
//
//        val park = DRIVE.trajectorySequenceBuilder(toBoard.end())
//                .addDisplacementMarker(){
//                    LG.position = 0.1
//                }
//                .back(14.0)
//                .UNSTABLE_addTemporalMarkerOffset(1.0) {
//                    LIFT.velocity = 0.0
//                    LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
//                    LG.position = 0.0
//                    WRIST.position = TestVars.WristLevelPos
//                    LIFT.targetPosition = 0
//                    LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
//                    LIFT.velocity = 800.0
//                }
//                .lineTo(Vector2d(42.0, color*RETURN_POS))
//                .UNSTABLE_addTemporalMarkerOffset(1.0){
//                    LIFT.velocity = 0.0
//                    LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
//                }
//                .forward(24.0)
//                .build()
//
//        DRIVE.followTrajectorySequence(park)
//
//        stop()
//    }
//}

