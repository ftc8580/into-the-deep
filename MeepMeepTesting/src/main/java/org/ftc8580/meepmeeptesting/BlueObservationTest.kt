package org.ftc8580.meepmeeptesting

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.rowlandhall.meepmeep.MeepMeep
import org.rowlandhall.meepmeep.MeepMeep.Background
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder
import org.rowlandhall.meepmeep.roadrunner.DriveShim
import org.rowlandhall.meepmeep.roadrunner.DriveTrainType

fun main(args: Array<String>) {
    System.setProperty("sun.java2d.opengl", "true")
    val meepMeep = MeepMeep(800)

    val rotationPickup = Math.toRadians(270.0)
    val rotationDelivery = Math.toRadians(90.0)
    val pickupY = 63.5
    val prePickupY = 58.0
    val deliveryY = 33.5
    val pickupX = -34.0
    val pushY = 15.0
    val prePickupPose = Pose2d(pickupX, prePickupY, rotationPickup)
    val pickupPose = Pose2d(pickupX, pickupY, rotationPickup)
    val deliveryStartingX = -16.0

    val myBot =
        DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setDriveTrainType(DriveTrainType.MECANUM)
            .setDimensions(14.0, 17.0)
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 14.0)
            .followTrajectorySequence { drive: DriveShim ->
                drive.trajectorySequenceBuilder(
                    Pose2d(
                        -16.0,
                        pickupY,
                        rotationDelivery
                    )
                )
                    .lineToLinearHeading(Pose2d(deliveryStartingX, deliveryY, rotationDelivery)) // Deliver preloaded specimen
                    .waitSeconds(1.0)
                    .lineToLinearHeading(Pose2d(deliveryStartingX, deliveryY + 4.0, rotationDelivery))
                    .lineToLinearHeading(Pose2d(-36.0, deliveryY + 4.0, rotationPickup))
                    .lineToLinearHeading(Pose2d(-36.0, pushY, rotationPickup))
                    .lineToLinearHeading(Pose2d(-48.0, pushY, rotationPickup))
                    .lineToLinearHeading(Pose2d(-48.0, 54.0, rotationPickup))
                    .lineToLinearHeading(prePickupPose)
                    .lineToLinearHeading(pickupPose) // Pickup new specimen
                    .waitSeconds(0.5)
                    .lineToLinearHeading(Pose2d(deliveryStartingX + 2.0, deliveryY, rotationDelivery)) // Deliver specimen
                    .waitSeconds(1.0)
                    .lineToLinearHeading(prePickupPose)
                    .lineToLinearHeading(pickupPose) // Pickup new specimen
                    .waitSeconds(0.5)
                    .lineToLinearHeading(Pose2d(deliveryStartingX + 4.0, deliveryY, rotationDelivery)) // Deliver specimen
                    .waitSeconds(1.0)
                    .lineToLinearHeading(prePickupPose)
                    .lineToLinearHeading(pickupPose) // Pickup new specimen
                    .waitSeconds(0.5)
                    .lineToLinearHeading(Pose2d(deliveryStartingX + 6.0, deliveryY, rotationDelivery)) // Deliver specimen
                    .waitSeconds(1.0)
                    .lineToLinearHeading(Pose2d(deliveryStartingX, deliveryY + 4.0, rotationDelivery))
                    .lineToLinearHeading(Pose2d(-36.0, deliveryY + 4.0, rotationPickup))
                    .lineToLinearHeading(Pose2d(-36.0, pushY, rotationPickup))
                    .lineToLinearHeading(Pose2d(-48.0, pushY, rotationPickup))
                    .lineToLinearHeading(Pose2d(-58.0, pushY, rotationPickup))
//                    .lineToLinearHeading(Pose2d(-58.0, 54.0, rotationPickup))
//                    .lineToLinearHeading(Pose2d(-58.0, pushY, rotationPickup))
//                    .lineToLinearHeading(Pose2d(-63.0, pushY, rotationPickup))
//                    .lineToLinearHeading(Pose2d(-63.0, 54.0, rotationPickup))
                    .build()
            }


    meepMeep.setBackground(Background.FIELD_INTOTHEDEEP_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start()
}