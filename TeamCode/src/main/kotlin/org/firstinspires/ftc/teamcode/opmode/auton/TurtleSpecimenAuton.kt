package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.actions.GripperPosition
import org.firstinspires.ftc.teamcode.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.hardware.HardwareManager
import org.firstinspires.ftc.teamcode.subsystem.GripperHeight
import org.firstinspires.ftc.teamcode.subsystem.GripperSubsystem

@Suppress("Unused")
@Autonomous(name = "Turtle Specimen", group = "Turtle")
class TurtleSpecimenAuton : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val initialPose = Pose2d(-8.0, 63.5, Math.toRadians(270.0))
        val hardware = HardwareManager(hardwareMap)
        val drive = MecanumDrive(hardware, initialPose)

        val gripperSubsystem = GripperSubsystem(hardware)

        val deliveryPositionBuilder = drive.actionBuilder(initialPose)
            .lineToY(34.0) // TODO: Fix this value

        val parkPositionBuilder = deliveryPositionBuilder.fresh()
            .splineToConstantHeading(
                Vector2d(-60.0, 60.0),
                Math.toRadians(270.0)
            )

        waitForStart()

        if (isStopRequested) return

        runBlocking(
            SequentialAction(
                GripperPosition(gripperSubsystem, GripperHeight.HIGH, 500.0), // Raise the gripper to the delivery position
                deliveryPositionBuilder.build(), // Drive up to the submersible
                GripperPosition(gripperSubsystem, GripperHeight.HOME), // Lower gripper to deliver specimen
                parkPositionBuilder.build() // Drive to park position
            )
        )
    }
}