package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.HardwareManager
import org.firstinspires.ftc.teamcode.subsystem.internal.MotorGroupSubsystem

class ArmRotationSubsystem(hardware: HardwareManager) : MotorGroupSubsystem() {
    private val rotationMotorGroup = hardware.viperRotationMotorGroup
    private val rotationEncoder = hardware.armRotationEncoder

    init {
        rotationMotorGroup?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rotationMotorGroup?.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun correctRotationGroupFollower() {
        rotationMotorGroup?.correctFollower()
    }

    fun setRotationMotorGroupPower(power: Double) {
        if (rotationMotorGroup == null) return

        if (power == 0.0) {
            rotationMotorGroup.power = 0.0
            return
        }

        if (power < 0 && (currentPosition ?: 0) >= ArmRotationPosition.TOP.position) {
            rotationMotorGroup.power = 0.0
        } else if (power > 0 && (currentPosition ?: 1) <= ArmRotationPosition.HOME.position) {
            rotationMotorGroup.power = 0.0
        } else {
            rotationMotorGroup.power = getBoundedPower(power)
        }
    }

    fun rotateToPosition(input: ArmRotationPosition) {
        val safePosition = getBoundedPosition(
            input.position,
            ArmRotationPosition.HOME.position,
            ArmRotationPosition.TOP.position
        )

        rotationMotorGroup?.safelyGoToPosition(-safePosition, ROTATION_SPEED)
    }

    fun rotateHome() = rotateToPosition(ArmRotationPosition.HOME)

    fun rotateDrive() = rotateToPosition(ArmRotationPosition.DRIVE)

    fun rotateMax() = rotateToPosition(ArmRotationPosition.TOP)

    val currentPosition: Int?
        get() = rotationEncoder?.getPositionAndVelocity()?.position

    companion object {
        const val ROTATION_SPEED = 0.5
    }
}

enum class ArmRotationPosition(val position: Int) {
    HOME(0),
    AUTON_PICKUP(325),
    DRIVE(750),
    PARK(1650),
    TOP(3300)
}

// Pickup position - rotation = 325, extension = 2600