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

    val initialX = 40.0
    val initialY = 63.5
    val initialHeading = Math.toRadians(270.0)
    val deliveryXY = 52.0
    val deliveryTangentHeadingDegrees = 45.0
    val deliveryTangentHeading = Math.toRadians(deliveryTangentHeadingDegrees)
    val deliveryRobotHeading = Math.toRadians(deliveryTangentHeadingDegrees + 180.0)
    val finalDeliveryRobotHeading = Math.toRadians(deliveryTangentHeadingDegrees + 270.0)
    val intermediateHeadingDegrees = 90.0
    val pickupYStart = 37.0
    val pickupYEnd = 42.0
    val pickupX1 = 48.5
    val pickupX2 = pickupX1 + 10.0
    val parkX = 23.5
    val parkY = 12.0
    val parkHeading = Math.toRadians(180.0)

    val action = bot.drive.actionBuilder(Pose2d(initialX, initialY, initialHeading))
        .splineToConstantHeading(Vector2d(pickupX1, pickupYStart), initialHeading)
        .waitSeconds(1.0)
        .lineToY(pickupYEnd)
        .setTangent(intermediateHeadingDegrees)
        .splineToLinearHeading(Pose2d(deliveryXY, deliveryXY, deliveryRobotHeading), deliveryTangentHeading)
        .waitSeconds(1.0)
        .splineToLinearHeading(Pose2d(pickupX2, pickupYStart, initialHeading), initialHeading)
        .waitSeconds(1.0)
        .lineToY(pickupYEnd)
        .setTangent(intermediateHeadingDegrees)
        .splineToLinearHeading(Pose2d(deliveryXY, deliveryXY, deliveryRobotHeading), deliveryTangentHeading)
        .waitSeconds(1.0)
        .splineToLinearHeading(Pose2d(pickupX2, pickupYStart, finalDeliveryRobotHeading), initialHeading)
        .waitSeconds(1.0)
        .lineToY(pickupYEnd)
        .setTangent(deliveryTangentHeadingDegrees)
        .splineToLinearHeading(Pose2d(deliveryXY, deliveryXY, deliveryRobotHeading), deliveryTangentHeading)
        .waitSeconds(1.0)
        .setTangent(deliveryRobotHeading)
        .splineToLinearHeading(Pose2d(parkX, parkY, parkHeading), parkHeading)
        .build()

    bot.runAction(action)

    meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(bot)
        .start()
}