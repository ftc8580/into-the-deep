package org.ftc8580.meepmeeptesting

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveTrainType

fun main(args: Array<String>) {
    System.setProperty("sun.java2d.opengl", "true")
    val meepMeep = MeepMeep(800)

    val bot = DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        .setDriveTrainType(DriveTrainType.MECANUM)
        .setDimensions(14.0, 17.0)
        .setConstraints(60.0, 60.0, Math.PI, Math.PI, 13.0)
        .build()

    val action = bot.drive.actionBuilder(
        Pose2d(-8.0, 63.0, Math.toRadians(270.0))
    )
        .waitSeconds(1.0) // Wait for gripper to raise
        .lineToY(33.0) // Drive to first delivery position
        .waitSeconds(0.25) // Wait for gripper to deliver
        .setTangent(Math.toRadians(150.0)) // Set exit direction
        .splineToConstantHeading(Vector2d(-37.0, 24.0), Math.toRadians(270.0)) // Spline around submersible corner
        .setTangent(Math.toRadians(270.0)) // Continue moving in the correct direction
        .splineToConstantHeading(Vector2d(-47.5, 12.0), Math.toRadians(90.0)) // Continue spline to position behind first sample
        .splineToConstantHeading(Vector2d(-47.5, 53.0), Math.toRadians(90.0)) // Push first sample to observation zone
        .setTangent(Math.toRadians(90.0))
        .splineToConstantHeading(Vector2d(-50.5, 53.0), Math.toRadians(270.0)) // Make a short loop to head back to pickup
        .splineToConstantHeading(Vector2d(-50.5, 12.0), Math.toRadians(270.0)) // Drive back to next pickup
        .setTangent(Math.toRadians(270.0))
        .splineToConstantHeading(Vector2d(-57.5, 16.0), Math.toRadians(90.0)) // Make a short loop to position behind second sample
        .splineToConstantHeading(Vector2d(-57.5, 53.0), Math.toRadians(90.0)) // Push second sample to the observation area
//        .setTangent(Math.toRadians(90.0))
//        .splineToConstantHeading(Vector2d(-59.5, 53.0), Math.toRadians(270.0)) // Make a short loop to head back to pickup
//        .splineToConstantHeading(Vector2d(-59.5, 12.0), Math.toRadians(270.0)) // Drive back to next pickup
//        .setTangent(Math.toRadians(270.0))
//        .splineToConstantHeading(Vector2d(-63.0, 16.0), Math.toRadians(90.0)) // Make a short loop to position behind third sample
//        .splineToConstantHeading(Vector2d(-63.0, 53.0), Math.toRadians(90.0)) // Push third sample to the observation area
        .splineToLinearHeading(Pose2d(-40.0, 48.0, Math.toRadians(90.0)), Math.toRadians(90.0))
        .waitSeconds(1.0)
        .splineToConstantHeading(Vector2d(-40.0, 63.0), Math.toRadians(90.0))
        .waitSeconds(0.4)
        .setTangent(-45.0)
        .splineToSplineHeading(Pose2d(-6.0, 33.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
        .waitSeconds(0.25)
        .setTangent(Math.toRadians(90.0))
        .splineToLinearHeading(Pose2d(-40.0, 48.0, Math.toRadians(90.0)), Math.toRadians(90.0))
        .waitSeconds(1.0)
        .setTangent(Math.toRadians(90.0))
        .splineToLinearHeading(Pose2d(-40.0, 56.0, Math.toRadians(90.0)), Math.toRadians(90.0))
        .splineToConstantHeading(Vector2d(-40.0, 63.0), Math.toRadians(90.0))
        .waitSeconds(0.4)
        .setTangent(-45.0)
        .splineToSplineHeading(Pose2d(-4.0, 33.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
        .waitSeconds(0.25)
        .setTangent(Math.toRadians(90.0))
        .splineToLinearHeading(Pose2d(-40.0, 48.0, Math.toRadians(90.0)), Math.toRadians(90.0))
        .waitSeconds(1.0)
        .splineToConstantHeading(Vector2d(-40.0, 63.0), Math.toRadians(90.0))
        .waitSeconds(0.4)
        .setTangent(-45.0)
        .splineToSplineHeading(Pose2d(-2.0, 33.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
        .waitSeconds(0.25)
        .setTangent(Math.toRadians(90.0))
        .splineToConstantHeading(Vector2d(-56.0, 58.0), Math.toRadians(180.0))
        .build()

    bot.runAction(action)

    meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(bot)
        .start()
}