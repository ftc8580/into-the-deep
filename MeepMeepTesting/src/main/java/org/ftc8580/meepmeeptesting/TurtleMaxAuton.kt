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
        .waitSeconds(1.2)
        .lineToY(33.0)
        .waitSeconds(1.0)
        .lineToY(38.0)
        .setTangent(Math.toRadians(90.0))
        .splineToConstantHeading(Vector2d(-37.0, 22.0), Math.toRadians(270.0))
//        .lineToY(14.0)
//        .strafeToConstantHeading(Vector2d(-47.5, 14.0))
        .setTangent(Math.toRadians(270.0))
        .splineToConstantHeading(Vector2d(-47.5, 14.0), Math.toRadians(90.0))
        .strafeToConstantHeading(Vector2d(-47.5, 53.0))
        .strafeToConstantHeading(Vector2d(-47.5, 14.0))
        .strafeToConstantHeading(Vector2d(-57.5, 14.0))
        .strafeToConstantHeading(Vector2d(-57.5, 53.0))
        .strafeToConstantHeading(Vector2d(-57.5, 45.0)) // Remove if uncommenting the other lines
        .waitSeconds(0.25)
//            .strafeToConstantHeading(Vector2d(-57.5, 12.0))
//            .strafeToConstantHeading(Vector2d(-63.0, 12.0))
//            .strafeToConstantHeading(Vector2d(-63.0, 53.0))
//            .strafeToConstantHeading(Vector2d(-63.0, 48.0))
        .strafeToLinearHeading(Vector2d(-40.0, 58.0), Math.toRadians(90.0))
        .waitSeconds(1.0)
        .strafeToConstantHeading(Vector2d(-40.0, 63.0))
        .waitSeconds(0.5)
        .setTangent(0.0)
        .splineToSplineHeading(Pose2d(-7.0, 33.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
        .waitSeconds(0.5)
        .strafeToConstantHeading(Vector2d(-7.0, 38.0))
        .setTangent(Math.toRadians(90.0))
        .splineToLinearHeading(Pose2d(-40.0, 58.0, Math.toRadians(90.0)), Math.toRadians(90.0))
        .waitSeconds(1.0)
        .strafeToConstantHeading(Vector2d(-40.0, 63.0))
        .waitSeconds(0.5)
//        .strafeToConstantHeading(Vector2d(-6.0, 38.0))
        .setTangent(0.0)
        .splineToSplineHeading(Pose2d(-6.0, 33.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
        .waitSeconds(0.5)
        .setTangent(Math.toRadians(90.0))
        .splineToLinearHeading(Pose2d(-40.0, 58.0, Math.toRadians(90.0)), Math.toRadians(90.0))
        .build()

    bot.runAction(action)

    meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(bot)
        .start()
}