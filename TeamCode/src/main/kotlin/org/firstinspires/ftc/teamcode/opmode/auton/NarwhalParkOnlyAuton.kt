package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.actions.buildArmPositionAction
import org.firstinspires.ftc.teamcode.opmode.AutonBase
import org.firstinspires.ftc.teamcode.subsystem.ArmExtensionPosition
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationPosition
import org.firstinspires.ftc.teamcode.subsystem.WristRotationPosition

@Suppress("Unused")
@Autonomous(name = "Narwhal Park Only", group = "Narwhal")
class NarwhalParkOnlyAuton : AutonBase() {
    private val initialPose = Pose2d(40.0, 63.5, Math.toRadians(270.0))

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initialize(initialPose)

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

        val action = drive.actionBuilder(initialPose)
            .strafeToConstantHeading(Vector2d(36.0, 12.0))
            .turnTo(Math.toRadians(180.0))
            .afterTime(0.0, armToPreParkPositionAction) // Move arm to pre-park position while driving
            .strafeToConstantHeading(Vector2d(23.5, 12.0))
            .afterTime(0.0, armToParkPositionAction) // Move arm to contact rung
            .waitSeconds(2.0) // Ensure wait until arm is moved
            .build()

        waitForStart()

        if (isStopRequested) return

        runBlocking(action)
    }
}