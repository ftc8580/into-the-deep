package org.firstinspires.ftc.teamcode.odometry.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig
import com.acmerobotics.dashboard.config.variable.CustomVariable
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.AngularRampLogger
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger
import com.acmerobotics.roadrunner.ftc.DriveType
import com.acmerobotics.roadrunner.ftc.DriveView
import com.acmerobotics.roadrunner.ftc.DriveViewFactory
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.ForwardPushTest
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger
import com.acmerobotics.roadrunner.ftc.LateralPushTest
import com.acmerobotics.roadrunner.ftc.LateralRampLogger
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.teamcode.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.hardware.HardwareManager
import org.firstinspires.ftc.teamcode.odometry.ThreeDeadWheelLocalizer

class TuningOpModes {
    companion object {
        val DRIVE_CLASS: Class<*> = MecanumDrive::class.java

        private const val GROUP: String = "quickstart"
        private const val DISABLED: Boolean = false

        private fun metaForClass(cls: Class<out OpMode?>): OpModeMeta {
            return OpModeMeta.Builder()
                .setName(cls.simpleName)
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build()
        }

        @OpModeRegistrar
        @JvmStatic
        fun register(manager: OpModeManager) {
            if (DISABLED) return

            val dvf: DriveViewFactory
            if (DRIVE_CLASS == MecanumDrive::class.java) {
                dvf = object : DriveViewFactory {
                    override fun make(h: HardwareMap): DriveView {
                        val hardware = HardwareManager(h)
                        val md = MecanumDrive(hardware, Pose2d(0.0, 0.0, 0.0))
                        val leftEncs: MutableList<Encoder> = ArrayList()
                        val rightEncs: MutableList<Encoder> = ArrayList()
                        val parEncs: MutableList<Encoder> = ArrayList()
                        val perpEncs: MutableList<Encoder> = ArrayList()
                        when (md.localizer) {
                            is ThreeDeadWheelLocalizer -> {
                                val dl: ThreeDeadWheelLocalizer = md.localizer
                                parEncs.add(dl.par0)
                                parEncs.add(dl.par1)
                                perpEncs.add(dl.perp)
                            }

                            else -> {
                                throw RuntimeException("unknown localizer")
                            }
                        }
                        return DriveView(
                            DriveType.MECANUM,
                            MecanumDrive.PARAMS.inPerTick,
                            MecanumDrive.PARAMS.maxWheelVel,
                            MecanumDrive.PARAMS.minProfileAccel,
                            MecanumDrive.PARAMS.maxProfileAccel,
                            h.getAll(LynxModule::class.java),
                            listOf(
                                hardware.leftFrontMotor,
                                hardware.leftRearMotor
                            ),
                            listOf(
                                hardware.rightFrontMotor,
                                hardware.rightRearMotor
                            ),
                            leftEncs,
                            rightEncs,
                            parEncs,
                            perpEncs,
                            hardware.lazyImu,
                            hardware.batteryVoltageSensor
                        ) {
                            MotorFeedforward(
                                MecanumDrive.PARAMS.kS,
                                MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                                MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick
                            )
                        }
                    }
                }
            } else {
                throw RuntimeException()
            }

            manager.register(metaForClass(AngularRampLogger::class.java), AngularRampLogger(dvf))
            manager.register(metaForClass(ForwardPushTest::class.java), ForwardPushTest(dvf))
            manager.register(metaForClass(ForwardRampLogger::class.java), ForwardRampLogger(dvf))
            manager.register(metaForClass(LateralPushTest::class.java), LateralPushTest(dvf))
            manager.register(metaForClass(LateralRampLogger::class.java), LateralRampLogger(dvf))
            manager.register(
                metaForClass(ManualFeedforwardTuner::class.java),
                ManualFeedforwardTuner(dvf)
            )
            manager.register(
                metaForClass(MecanumMotorDirectionDebugger::class.java),
                MecanumMotorDirectionDebugger(dvf)
            )
            manager.register(
                metaForClass(DeadWheelDirectionDebugger::class.java),
                DeadWheelDirectionDebugger(dvf)
            )

            manager.register(
                metaForClass(ManualFeedbackTuner::class.java),
                ManualFeedbackTuner::class.java
            )
            manager.register(metaForClass(SplineTest::class.java), SplineTest::class.java)
            manager.register(
                metaForClass(LocalizationTest::class.java),
                LocalizationTest::class.java
            )

            FtcDashboard.getInstance().withConfigRoot { configRoot: CustomVariable ->
                for (c in listOf<Class<out Any?>>(
                    AngularRampLogger::class.java,
                    ForwardRampLogger::class.java,
                    LateralRampLogger::class.java,
                    ManualFeedforwardTuner::class.java,
                    MecanumMotorDirectionDebugger::class.java,
                    ManualFeedbackTuner::class.java
                )) {
                    configRoot.putVariable(c.simpleName, ReflectionConfig.createVariableFromClass(c))
                }
            }
        }
    }
}