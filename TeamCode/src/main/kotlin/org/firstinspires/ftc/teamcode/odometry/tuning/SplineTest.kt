package org.firstinspires.ftc.teamcode.odometry.tuning

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.hardware.HardwareManager

class SplineTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val beginPose: Pose2d = Pose2d(0.0, 0.0, 0.0)
        if (TuningOpModes.DRIVE_CLASS == MecanumDrive::class.java) {
            val drive: MecanumDrive = MecanumDrive(HardwareManager(hardwareMap), beginPose)

            waitForStart()

            runBlocking(
                drive.actionBuilder(beginPose)
                    .splineTo(Vector2d(30.0, 30.0), Math.PI / 2)
                    .splineTo(Vector2d(0.0, 60.0), Math.PI)
                    .build()
            )
        } else {
            throw RuntimeException()
        }
    }
}