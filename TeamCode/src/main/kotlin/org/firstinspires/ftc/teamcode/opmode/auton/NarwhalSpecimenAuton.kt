package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.actions.GripperPosition
import org.firstinspires.ftc.teamcode.actions.RotationPosition
import org.firstinspires.ftc.teamcode.actions.buildArmPositionAction
import org.firstinspires.ftc.teamcode.opmode.AutonBase
import org.firstinspires.ftc.teamcode.subsystem.ArmExtensionPosition
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationPosition
import org.firstinspires.ftc.teamcode.subsystem.GripperHeight
import org.firstinspires.ftc.teamcode.subsystem.WristRotationPosition

@Suppress("Unused")
@Autonomous(name = "Narwhal Single Specimen", group = "Narwhal")
class NarwhalSpecimenAuton : AutonBase() {
    private val initialPose = Pose2d(40.0, 63.5, Math.toRadians(270.0))

    private val armToPreParkPositionAction = buildArmPositionAction(
        armExtensionSubsystem,
        armRotationSubsystem,
        activeIntakeSubsystem,
        ArmExtensionPosition.HOME,
        ArmRotationPosition.DRIVE,
        WristRotationPosition.PICKUP
    )

    private val armToParkPositionAction = buildArmPositionAction(
        armExtensionSubsystem,
        armRotationSubsystem,
        activeIntakeSubsystem,
        ArmExtensionPosition.HOME,
        ArmRotationPosition.PARK,
        WristRotationPosition.DELIVER
    )

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initialize(initialPose)

        val action = drive.actionBuilder(initialPose)
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH)) // Raise gripper to delivery position
            .splineToConstantHeading(Vector2d(5.0, 33.0), Math.toRadians(270.0)) // Spline to specimen delivery position
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME)) // Hang specimen
            .waitSeconds(0.3) // Wait for specimen to hook
            .setTangent(Math.toRadians(90.0)) // Set exit heading
            .splineToLinearHeading(Pose2d(28.0, 40.0, Math.toRadians(200.0)), Math.toRadians(0.0)) // Intermediate pose around submersible
            .afterTime(0.2, armToPreParkPositionAction) // Move arm to pre-park position while driving
            .splineToLinearHeading(Pose2d(23.5, 12.0, Math.toRadians(180.0)), Math.toRadians(180.0)) // Finish drive into park position
            .afterTime(0.0, armToParkPositionAction) // Move arm to contact rung
            .waitSeconds(2.0) // Ensure wait until arm is moved
            .build()

        waitForStart()

        if (isStopRequested) return

        runBlocking(action)
    }
}