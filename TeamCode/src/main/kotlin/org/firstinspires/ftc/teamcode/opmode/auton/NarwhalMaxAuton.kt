package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.actions.DeliverSample
import org.firstinspires.ftc.teamcode.actions.GripperPosition
import org.firstinspires.ftc.teamcode.actions.IntakeSample
import org.firstinspires.ftc.teamcode.actions.buildArmPositionAction
import org.firstinspires.ftc.teamcode.opmode.AutonBase
import org.firstinspires.ftc.teamcode.subsystem.ArmExtensionPosition
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationPosition
import org.firstinspires.ftc.teamcode.subsystem.GripperHeight
import org.firstinspires.ftc.teamcode.subsystem.WristRotationPosition

@Suppress("Unused")
@Autonomous(name = "Narwhal Max", group = "Narwhal")
class NarwhalMaxAuton : AutonBase() {

    private val initialX = 40.0
    private val initialY = 63.5
    private val initialHeading = Math.toRadians(270.0)
    private val deliveryXY = 52.0
    private val deliveryTangentHeadingDegrees = 45.0
    private val deliveryTangentHeading = Math.toRadians(deliveryTangentHeadingDegrees)
    private val deliveryRobotHeading = Math.toRadians(deliveryTangentHeadingDegrees + 180.0)
    private val finalDeliveryRobotHeading = Math.toRadians(deliveryTangentHeadingDegrees + 270.0)
    private val intermediateHeadingDegrees = 90.0
    private val pickupYStart = 37.0
    private val pickupYEnd = 42.0
    private val pickupX1 = 48.5
    private val pickupX2 = pickupX1 + 10.0
    private val parkX = 23.5
    private val parkY = 12.0
    private val parkHeading = Math.toRadians(180.0)

    private val initialPose = Pose2d(initialX, initialY, initialHeading)
    private val deliveryPose = Pose2d(deliveryXY, deliveryXY, deliveryRobotHeading)

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initialize(initialPose)

        val armToDrivePositionAction = buildArmPositionAction(
            armExtensionSubsystem,
            armRotationSubsystem,
            activeIntakeSubsystem,
            ArmExtensionPosition.AUTON_PICKUP,
            ArmRotationPosition.DRIVE,
            WristRotationPosition.PICKUP
        )

        val armToPickupPositionAction = buildArmPositionAction(
            armExtensionSubsystem,
            armRotationSubsystem,
            activeIntakeSubsystem,
            ArmExtensionPosition.AUTON_PICKUP,
            ArmRotationPosition.AUTON_PICKUP,
            WristRotationPosition.PICKUP
        )

        val armToDeliveryPositionAction = buildArmPositionAction(
            armExtensionSubsystem,
            armRotationSubsystem,
            activeIntakeSubsystem,
            ArmExtensionPosition.MAX_UP,
            ArmRotationPosition.TOP,
            WristRotationPosition.DELIVER
        )

        val armToPreParkPositionAction = buildArmPositionAction(
            armExtensionSubsystem,
            armRotationSubsystem,
            activeIntakeSubsystem,
            ArmExtensionPosition.HOME,
            ArmRotationPosition.DRIVE,
            WristRotationPosition.PICKUP
        )

        val armToParkPositionAction = buildArmPositionAction(
            armExtensionSubsystem,
            armRotationSubsystem,
            activeIntakeSubsystem,
            ArmExtensionPosition.HOME,
            ArmRotationPosition.PARK,
            WristRotationPosition.PICKUP
        )

        val action = drive.actionBuilder(initialPose) // Starting position
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH)) // Raise gripper
            .waitSeconds(0.5) // Wait long enough for gripper to reach max position before arriving at submersible
            .splineToConstantHeading(Vector2d(5.0, 33.0), Math.toRadians(270.0)) // Drive to submersible delivery position
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME)) // Lower gripper
            .waitSeconds(0.3) // Wait long enough to hook specimen before driving away
            .setTangent(Math.toRadians(90.0)) // Start spline in 90deg direction
            // DELIVER FIRST SAMPLE
            .splineToConstantHeading(Vector2d(pickupX1, pickupYStart), initialHeading) // Spline to first pickup position
            .afterTime(0.0, armToPickupPositionAction) // Get ready to pick up the sample
            .waitSeconds(0.75) // Wait for the arm to get in the pickup position
            .afterTime(0.0, IntakeSample(activeIntakeSubsystem)) // Start intake servos
            .lineToY(pickupYEnd) // Move backwards to pull in the sample
            .afterTime(0.0, armToDeliveryPositionAction) // Move the arm into the delivery position while driving
            .setTangent(intermediateHeadingDegrees) // Leave the pickup position in the right direction
            .splineToLinearHeading(deliveryPose, deliveryTangentHeading) // Spline to the delivery position
            .afterTime(0.0, DeliverSample(activeIntakeSubsystem)) // Eject the sample from the intake
            .waitSeconds(1.0) // Wait for the sample to go in the basket
            .afterTime(0.0, armToDrivePositionAction) // After delivery, return to drive position while driving to next pickup
            // DELIVER SECOND SAMPLE
            .splineToLinearHeading(Pose2d(pickupX2, pickupYStart, initialHeading), initialHeading) // Spline to the second pickup position
            .afterTime(0.0, armToPickupPositionAction) // Get ready to pick up the sample
            .waitSeconds(0.75) // Wait for the arm to get in the pickup position
            .afterTime(0.0, IntakeSample(activeIntakeSubsystem)) // Start intake servos
            .lineToY(pickupYEnd) // Move backwards to pull in the sample
            .setTangent(intermediateHeadingDegrees) // Leave the pickup position in the right direction
            .splineToLinearHeading(deliveryPose, deliveryTangentHeading) // Spline to the delivery position
            .afterTime(0.0, DeliverSample(activeIntakeSubsystem)) // Eject the sample from the intake
            .waitSeconds(1.0) // Wait for the sample to go in the basket
            .afterTime(0.0, armToDrivePositionAction) // After delivery, return to drive position while driving to next pickup
            // DELIVER THIRD SAMPLE
            .splineToLinearHeading(Pose2d(pickupX2, pickupYStart, finalDeliveryRobotHeading), initialHeading) // Go to the final pickup position
            .afterTime(0.0, armToPickupPositionAction) // Get ready to pick up the sample
            .waitSeconds(0.75) // Wait for the arm to get in the pickup position
            .afterTime(0.0, IntakeSample(activeIntakeSubsystem)) // Start intake servos
            .lineToY(pickupYEnd) // Move backwards to pull in the sample
            .setTangent(deliveryTangentHeadingDegrees) // Leave the pickup position in the right direction
            .splineToLinearHeading(deliveryPose, deliveryTangentHeading) // Spline to the delivery position
            .afterTime(0.0, DeliverSample(activeIntakeSubsystem)) // Eject the sample from the intake
            .waitSeconds(1.0) // Wait for the sample to go in the basket
            // PARK
            .afterTime(0.0, armToPreParkPositionAction) // After delivery, go to pre-park position
            .setTangent(deliveryRobotHeading) // Leave the pickup position in the right direction
            .splineToLinearHeading(Pose2d(parkX, parkY, parkHeading), parkHeading) // Spline to the park position
            .afterTime(0.0, armToParkPositionAction) // Move arm to touch submersible for parking points
            .waitSeconds(1.0) // Wait for arm movement (just in case)
            .build()

        waitForStart()

        if (isStopRequested) return

        runBlocking(action)
    }
}