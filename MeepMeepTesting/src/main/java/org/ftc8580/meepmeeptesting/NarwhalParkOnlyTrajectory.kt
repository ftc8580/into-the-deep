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
        Pose2d(40.0, 63.5, Math.toRadians(270.0))
    )
        .strafeToConstantHeading(Vector2d(36.0, 12.0))
        .turnTo(Math.toRadians(180.0))
        .strafeToConstantHeading(Vector2d(23.5, 12.0))
        .build()

    bot.runAction(action)

    meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(bot)
        .start()
}