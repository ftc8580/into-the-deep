package org.firstinspires.ftc.teamcode.odometry.tuning

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.config.LocalizerParams
import org.firstinspires.ftc.teamcode.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.hardware.HardwareManager
import org.firstinspires.ftc.teamcode.odometry.ThreeDeadWheelLocalizer

class ManualFeedbackTuner : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        if (TuningOpModes.DRIVE_CLASS == MecanumDrive::class.java) {
            val drive = MecanumDrive(HardwareManager(hardwareMap), Pose2d(0.0, 0.0, 0.0))

            if (drive.localizer is ThreeDeadWheelLocalizer) {
                if (LocalizerParams.perpXTicks == 0.0 && LocalizerParams.par0YTicks == 0.0 && LocalizerParams.par1YTicks == 1.0) {
                    throw RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.")
                }
            }
            waitForStart()

            while (opModeIsActive()) {
                runBlocking(
                    drive.actionBuilder(Pose2d(0.0, 0.0, 0.0))
                        .lineToX(DISTANCE)
                        .lineToX(0.0)
                        .build()
                )
            }
        } else {
            throw RuntimeException()
        }
    }

    companion object {
        var DISTANCE: Double = 64.0
    }
}