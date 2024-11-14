package org.firstinspires.ftc.teamcode.odometry

import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.Vector2dDual
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.config.LocalizerParams
import org.firstinspires.ftc.teamcode.hardware.HardwareManager

class ThreeDeadWheelLocalizer(hardware: HardwareManager, private val inPerTick: Double) : Localizer {

//    init {
//        write("THREE_DEAD_WHEEL_LocalizerParams", LocalizerParams.Companion)
//    }

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
                        (LocalizerParams.par0YTicks * par1PosDelta - LocalizerParams.par1YTicks * par0PosDelta) / (LocalizerParams.par0YTicks - LocalizerParams.par1YTicks),
                        (LocalizerParams.par0YTicks * par1PosVel.velocity - LocalizerParams.par1YTicks * par0PosVel.velocity) / (LocalizerParams.par0YTicks - LocalizerParams.par1YTicks),
                    )
                ).times(inPerTick),
                DualNum<Time>(
                    doubleArrayOf(
                        (LocalizerParams.perpXTicks / (LocalizerParams.par0YTicks - LocalizerParams.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                        (LocalizerParams.perpXTicks / (LocalizerParams.par0YTicks - LocalizerParams.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                    )
                ).times(inPerTick)
            ),
            DualNum(
                doubleArrayOf(
                    (par0PosDelta - par1PosDelta) / (LocalizerParams.par0YTicks - LocalizerParams.par1YTicks),
                    (par0PosVel.velocity - par1PosVel.velocity) / (LocalizerParams.par0YTicks - LocalizerParams.par1YTicks),
                )
            )
        )

        lastPar0Pos = par0PosVel.position
        lastPar1Pos = par1PosVel.position
        lastPerpPos = perpPosVel.position

        return twist
    }
}