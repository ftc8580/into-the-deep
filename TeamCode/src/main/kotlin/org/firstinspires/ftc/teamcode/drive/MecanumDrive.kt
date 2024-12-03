package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.now
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MecanumKinematics.WheelVelocities
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.VelConstraint
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.acmerobotics.roadrunner.ftc.FlightRecorder.write
import com.acmerobotics.roadrunner.range
import org.firstinspires.ftc.teamcode.config.MecanumDriveParams
import org.firstinspires.ftc.teamcode.hardware.HardwareManager
import org.firstinspires.ftc.teamcode.odometry.Localizer
import org.firstinspires.ftc.teamcode.odometry.ThreeDeadWheelLocalizer
import org.firstinspires.ftc.teamcode.util.Drawing.drawRobot
import java.util.LinkedList
import kotlin.math.ceil
import kotlin.math.max

class MecanumDrive(private val hardware: HardwareManager, pose: Pose2d) {
    val kinematics: MecanumKinematics = MecanumKinematics(
        MecanumDriveParams.inPerTick * MecanumDriveParams.trackWidthTicks, MecanumDriveParams.inPerTick / MecanumDriveParams.lateralInPerTick
    )

    val defaultTurnConstraints: TurnConstraints = TurnConstraints(
        MecanumDriveParams.maxAngVel, -MecanumDriveParams.maxAngAccel, MecanumDriveParams.maxAngAccel
    )
    val defaultVelConstraint: VelConstraint = MinVelConstraint(
        listOf(
            kinematics.WheelVelConstraint(MecanumDriveParams.maxWheelVel),
            AngularVelConstraint(MecanumDriveParams.maxAngVel)
        )
    )
    val defaultAccelConstraint: AccelConstraint =
        ProfileAccelConstraint(MecanumDriveParams.minProfileAccel, MecanumDriveParams.maxProfileAccel)

    val localizer: Localizer
    var pose: Pose2d

    private val poseHistory: LinkedList<Pose2d> = LinkedList<Pose2d>()

    private val estimatedPoseWriter = DownsampledWriter("ESTIMATED_POSE", 50000000)
    private val targetPoseWriter = DownsampledWriter("TARGET_POSE", 50000000)
    private val driveCommandWriter = DownsampledWriter("DRIVE_COMMAND", 50000000)
    private val mecanumCommandWriter = DownsampledWriter("MECANUM_COMMAND", 50000000)

    init {
        this.pose = pose

        localizer = ThreeDeadWheelLocalizer(hardware, MecanumDriveParams.inPerTick)

//        write("MECANUM_MecanumDriveParams", MecanumDriveParams.Companion)
    }

    fun setDrivePowers(powers: PoseVelocity2d?) {
        val wheelVels: WheelVelocities<Time> = MecanumKinematics(1.0).inverse(
            PoseVelocity2dDual.constant(powers!!, 1)
        )

        var maxPowerMag = 1.0
        for (power in wheelVels.all()) {
            maxPowerMag = max(maxPowerMag, power.value())
        }

        hardware.leftFrontMotor.power = wheelVels.leftFront[0] / maxPowerMag
        hardware.leftRearMotor.power = wheelVels.leftBack[0] / maxPowerMag
        hardware.rightRearMotor.power = wheelVels.rightBack[0] / maxPowerMag
        hardware.rightFrontMotor.power = wheelVels.rightFront[0] / maxPowerMag
    }

    inner class FollowTrajectoryAction(private val timeTrajectory: TimeTrajectory) : Action {
        private var beginTs = -1.0

        private val xPoints: DoubleArray
        private val yPoints: DoubleArray

        init {
            val disps = range(
                0.0, timeTrajectory.path.length(),
                max(2.0, ceil(timeTrajectory.path.length() / 2.0)).toInt()
            )
            xPoints = DoubleArray(disps.size)
            yPoints = DoubleArray(disps.size)
            for (i in disps.indices) {
                val p: Pose2d = timeTrajectory.path[disps[i], 1].value()
                xPoints[i] = p.position.x
                yPoints[i] = p.position.y
            }
        }

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double
            if (beginTs < 0) {
                beginTs = now()
                t = 0.0
            } else {
                t = now() - beginTs
            }

            if (t >= timeTrajectory.duration) {
                hardware.leftFrontMotor.power = 0.0
                hardware.leftRearMotor.power = 0.0
                hardware.rightRearMotor.power = 0.0
                hardware.rightFrontMotor.power = 0.0

                return false
            }

            val txWorldTarget: Pose2dDual<Time> = timeTrajectory[t]
//            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot = updatePoseEstimate()

            val command: PoseVelocity2dDual<Time> = HolonomicController(
                MecanumDriveParams.axialGain, MecanumDriveParams.lateralGain, MecanumDriveParams.headingGain,
                MecanumDriveParams.axialVelGain, MecanumDriveParams.lateralVelGain, MecanumDriveParams.headingVelGain
            )
                .compute(txWorldTarget, pose, robotVelRobot)
//            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels: WheelVelocities<Time> = kinematics.inverse(command)
            val voltage = hardware.batteryVoltageSensor.voltage

            val feedforward = MotorFeedforward(
                MecanumDriveParams.kS,
                MecanumDriveParams.kV / MecanumDriveParams.inPerTick, MecanumDriveParams.kA / MecanumDriveParams.inPerTick
            )
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
//            mecanumCommandWriter.write(
//                MecanumCommandMessage(
//                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
//                )
//            )

            hardware.leftFrontMotor.power = leftFrontPower
            hardware.leftRearMotor.power = leftBackPower
            hardware.rightRearMotor.power = rightBackPower
            hardware.rightFrontMotor.power = rightFrontPower

            p.put("x", pose.position.x)
            p.put("y", pose.position.y)
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()))

            val error: Pose2d = txWorldTarget.value().minusExp(pose)
            p.put("xError", error.position.x)
            p.put("yError", error.position.y)
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()))

            // only draw when active; only one drive action should be active at a time
            val c: Canvas = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            drawRobot(c, pose)

            c.setStroke("#4CAF50FF")
            c.setStrokeWidth(1)
            c.strokePolyline(xPoints, yPoints)

            return true
        }

        override fun preview(fieldOverlay: Canvas) {
            fieldOverlay.setStroke("#4CAF507A")
            fieldOverlay.setStrokeWidth(1)
            fieldOverlay.strokePolyline(xPoints, yPoints)
        }
    }

    inner class TurnAction(private val turn: TimeTurn) : Action {
        private var beginTs = -1.0

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double
            if (beginTs < 0) {
                beginTs = now()
                t = 0.0
            } else {
                t = now() - beginTs
            }

            if (t >= turn.duration) {
                hardware.leftFrontMotor.power = 0.0
                hardware.leftRearMotor.power = 0.0
                hardware.rightRearMotor.power = 0.0
                hardware.rightFrontMotor.power = 0.0

                return false
            }

            val txWorldTarget: Pose2dDual<Time> = turn[t]
//            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot = updatePoseEstimate()

            val command: PoseVelocity2dDual<Time> = HolonomicController(
                MecanumDriveParams.axialGain, MecanumDriveParams.lateralGain, MecanumDriveParams.headingGain,
                MecanumDriveParams.axialVelGain, MecanumDriveParams.lateralVelGain, MecanumDriveParams.headingVelGain
            )
                .compute(txWorldTarget, pose, robotVelRobot)
//            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels: WheelVelocities<Time> = kinematics.inverse(command)
            val voltage = hardware.batteryVoltageSensor.voltage
            val feedforward = MotorFeedforward(
                MecanumDriveParams.kS,
                MecanumDriveParams.kV / MecanumDriveParams.inPerTick, MecanumDriveParams.kA / MecanumDriveParams.inPerTick
            )
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
//            mecanumCommandWriter.write(
//                MecanumCommandMessage(
//                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
//                )
//            )

            hardware.leftFrontMotor.power = feedforward.compute(wheelVels.leftFront) / voltage
            hardware.leftRearMotor.power = feedforward.compute(wheelVels.leftBack) / voltage
            hardware.rightRearMotor.power = feedforward.compute(wheelVels.rightBack) / voltage
            hardware.rightFrontMotor.power = feedforward.compute(wheelVels.rightFront) / voltage

            val c: Canvas = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            drawRobot(c, pose)

            c.setStroke("#7C4DFFFF")
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)

            return true
        }

        override fun preview(fieldOverlay: Canvas) {
            fieldOverlay.setStroke("#7C4DFF7A")
            fieldOverlay.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)
        }
    }

    fun updatePoseEstimate(): PoseVelocity2d {
        val twist: Twist2dDual<Time> = localizer.update()
        pose = pose.plus(twist.value())

        val sensorPose = getEstimatedSensorLocation(pose)

        // Make sure the calculated pose from the distance sensors is non-null and different than the roadrunner pose before replacing
        if (sensorPose != null && !(pose.position.x == sensorPose.position.x && pose.position.y == sensorPose.position.y)) {
            pose = sensorPose
        }

        poseHistory.add(pose)
        while (poseHistory.size > 100) {
            poseHistory.removeFirst()
        }

//        estimatedPoseWriter.write(PoseMessage(pose))

        return twist.velocity().value()
    }

    private fun getEstimatedSensorLocation(roadrunnerPose: Pose2d): Pose2d? {
        val sensorReadings = mapOf(
            "front" to hardware.distanceSensors["front"]?.currentInches,
            "left" to hardware.distanceSensors["left"]?.currentInches,
            "right" to hardware.distanceSensors["right"]?.currentInches,
            "rear" to hardware.distanceSensors["rear"]?.currentInches
        )

        when (roadrunnerPose.heading.toDouble()) {
            in Math.toRadians(85.0)..Math.toRadians(95.0) -> { // Front is facing alliance wall
                return if (roadrunnerPose.position.x >= 0) { // Narwhal side
                    Pose2d(
                        Vector2d(
                            sensorReadings["right"]?.let {
                                72.0 - it - rightDistanceSensorOffset.y
                            } ?: roadrunnerPose.position.x,
                            sensorReadings["front"]?.let {
                                72.0 - it - frontDistanceSensorOffset.x
                            } ?: roadrunnerPose.position.y
                        ),
                        roadrunnerPose.heading
                    )
                } else { // Turtle side
                    Pose2d(
                        Vector2d(
                            sensorReadings["left"]?.let {
                                -72.0 + it + leftDistanceSensorOffset.y
                            } ?: roadrunnerPose.position.x,
                            sensorReadings["front"]?.let {
                                72.0 - it - frontDistanceSensorOffset.x
                            } ?: roadrunnerPose.position.y
                        ),
                        roadrunnerPose.heading
                    )
                }
            }
            in Math.toRadians(175.0)..Math.toRadians(185.0) -> { // Right side is facing alliance wall
                return if (roadrunnerPose.position.x >= 0) {
                    Pose2d(
                        Vector2d(
                            sensorReadings["rear"]?.let {
                                72.0 - it - rearDistanceSensorOffset.x
                            } ?: roadrunnerPose.position.x,
                            sensorReadings["right"]?.let {
                                72.0 - it - rightDistanceSensorOffset.y
                            } ?: roadrunnerPose.position.y
                        ),
                        roadrunnerPose.heading
                    )
                } else {
                    Pose2d(
                        Vector2d(
                            sensorReadings["front"]?.let {
                                -72.0 + it + rearDistanceSensorOffset.x
                            } ?: roadrunnerPose.position.x,
                            sensorReadings["right"]?.let {
                                72.0 - it - rightDistanceSensorOffset.y
                            } ?: roadrunnerPose.position.y
                        ),
                        roadrunnerPose.heading
                    )
                }
            }
            in Math.toRadians(265.0)..Math.toRadians(275.0) -> { // Rear is facing alliance wall
                return if (roadrunnerPose.position.x >= 0) {
                    Pose2d(
                        Vector2d(
                            sensorReadings["left"]?.let {
                                72.0 - it - leftDistanceSensorOffset.y
                            } ?: roadrunnerPose.position.x,
                            sensorReadings["rear"]?.let {
                                72.0 - it - rearDistanceSensorOffset.x
                            } ?: roadrunnerPose.position.y
                        ),
                        roadrunnerPose.heading
                    )
                } else {
                    Pose2d(
                        Vector2d(
                            sensorReadings["right"]?.let {
                                -72.0 + it + rightDistanceSensorOffset.y
                            } ?: roadrunnerPose.position.x,
                            sensorReadings["rear"]?.let {
                                72.0 - it - rearDistanceSensorOffset.x
                            } ?: roadrunnerPose.position.y
                        ),
                        roadrunnerPose.heading
                    )
                }
            }
            else -> return null
        }
    }

    private fun drawPoseHistory(c: Canvas) {
        val xPoints = DoubleArray(poseHistory.size)
        val yPoints = DoubleArray(poseHistory.size)

        for ((i, t) in poseHistory.withIndex()) {
            xPoints[i] = t.position.x
            yPoints[i] = t.position.y

        }

        c.setStrokeWidth(1)
        c.setStroke("#3F51B5")
        c.strokePolyline(xPoints, yPoints)
    }

    fun actionBuilder(beginPose: Pose2d): TrajectoryActionBuilder {
        return TrajectoryActionBuilder(
            { turn: TimeTurn -> TurnAction(turn) },
            { t: TimeTrajectory -> FollowTrajectoryAction(t) },
            TrajectoryBuilderParams(
                1e-6,
                ProfileParams(
                    0.25, 0.1, 1e-2
                )
            ),
            beginPose, 0.0,
            defaultTurnConstraints,
            defaultVelConstraint, defaultAccelConstraint
        )
    }

    companion object {
        // TODO: Fix these offsets and probably just make them Doubles
        private val frontDistanceSensorOffset = Vector2d(6.0, 0.0)
        private val leftDistanceSensorOffset = Vector2d(0.0, 5.0)
        private val rightDistanceSensorOffset = Vector2d(0.0, 5.0)
        private val rearDistanceSensorOffset = Vector2d(6.0, 0.0)
    }
}

