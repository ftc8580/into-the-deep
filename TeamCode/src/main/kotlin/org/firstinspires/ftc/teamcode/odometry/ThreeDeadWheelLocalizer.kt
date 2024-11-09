package org.firstinspires.ftc.teamcode.odometry

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.Vector2dDual
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.FlightRecorder.write
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.hardware.HardwareManager
import org.firstinspires.ftc.teamcode.messages.ThreeDeadWheelInputsMessage

class ThreeDeadWheelLocalizer(hardware: HardwareManager, val inPerTick: Double) : Localizer {
    @Config
    object LocalizerParams {
        @JvmField var par0YTicks: Double = -3091.9317284303575 // y position of the first parallel encoder (in tick units)
        @JvmField var par1YTicks: Double = 3083.104672695253 // y position of the second parallel encoder (in tick units)
        @JvmField var perpXTicks: Double = -2884.4319874095117 // x position of the perpendicular encoder (in tick units)
    }

    //        write("THREE_DEAD_WHEEL_PARAMS", PARAMS)

    // TODO: make sure your config has **motors** with these names (or change them)
    //   the encoders should be plugged into the slot matching the named motor
    //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
    var par0: Encoder = OverflowEncoder(RawEncoder(hardware.leftEncoder!!))
    var par1: Encoder = OverflowEncoder(RawEncoder(hardware.rightEncoder!!))
    var perp: Encoder = OverflowEncoder(RawEncoder(hardware.rearEncoder!!))

    private var lastPar0Pos = 0
    private var lastPar1Pos = 0
    private var lastPerpPos = 0
    private var initialized = false

    init {
        perp.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun update(): Twist2dDual<Time> {
        val par0PosVel: PositionVelocityPair = par0.getPositionAndVelocity()
        val par1PosVel: PositionVelocityPair = par1.getPositionAndVelocity()
        val perpPosVel: PositionVelocityPair = perp.getPositionAndVelocity()

//        write(
//            "THREE_DEAD_WHEEL_INPUTS",
//            ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel)
//        )

        if (!initialized) {
            initialized = true

            lastPar0Pos = par0PosVel.position
            lastPar1Pos = par1PosVel.position
            lastPerpPos = perpPosVel.position

            return Twist2dDual(
                Vector2dDual.constant(Vector2d(0.0, 0.0), 2),
                DualNum.constant(0.0, 2)
            )
        }

        val par0PosDelta = par0PosVel.position - lastPar0Pos
        val par1PosDelta = par1PosVel.position - lastPar1Pos
        val perpPosDelta = perpPosVel.position - lastPerpPos

        val twist: Twist2dDual<Time> = Twist2dDual(
            Vector2dDual(
                DualNum<Time>(
                    doubleArrayOf(
                        (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                    )
                ).times(inPerTick),
                DualNum<Time>(
                    doubleArrayOf(
                        (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                        (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                    )
                ).times(inPerTick)
            ),
            DualNum(
                doubleArrayOf(
                    (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                    (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                )
            )
        )

        lastPar0Pos = par0PosVel.position
        lastPar1Pos = par1PosVel.position
        lastPerpPos = perpPosVel.position

        return twist
    }

    companion object {
        var PARAMS = LocalizerParams
    }
}