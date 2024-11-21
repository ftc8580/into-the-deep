package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.HardwareManager
import org.firstinspires.ftc.teamcode.subsystem.internal.MotorGroupSubsystem

class ViperExtensionSubsystem(hardware: HardwareManager) : MotorGroupSubsystem() {
    private val rotationEncoder = hardware.armRotationEncoder
    private val extensionMotorGroup = hardware.viperExtensionMotorGroup
    private val extensionHomeSensor = hardware.extensionHomeSensor

    init {
        resetMotorEncoders()
        extensionMotorGroup?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun resetMotorEncoders() {
        extensionMotorGroup?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        extensionMotorGroup?.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun setExtensionMotorGroupPower(power: Double) {
        if (extensionMotorGroup == null) return

        if (power == 0.0) {
            extensionMotorGroup.power = 0.0
            return
        }

        if (extensionHomeSensor?.isPressed == true && power < 0) {
            extensionMotorGroup.power = 0.0
            return
        }

        // If encoder is not available, always allow maximum extension
        val rotationPosition = rotationEncoder?.getPositionAndVelocity()?.position ?: ROTATION_LIMIT_POSITION
        val maxExtensionPosition = if (rotationPosition < ROTATION_LIMIT_POSITION) {
            // Arm is below plane where max extension will break 42" limit
            ArmExtensionPosition.MAX_DOWN.position
        } else {
            // Arm is rotated up enough that we can extend to the maximum reach
            ArmExtensionPosition.MAX_UP.position
        }

        val currentPosition = extensionMotorGroup.getCurrentPosition()
        if (currentPosition >= maxExtensionPosition && power > 0) {
            extensionMotorGroup.power = 0.0
        } else {
            extensionMotorGroup.power = getBoundedPower(power)
        }
    }

    fun extendToPosition(target: ArmExtensionPosition) {
        val safePosition = getBoundedPosition(
            target.position,
            ArmExtensionPosition.HOME.position,
            ArmExtensionPosition.MAX_UP.position
        )

        extensionMotorGroup?.safelyGoToPosition(safePosition, EXTENSION_SPEED)
    }

    fun extendLowerBasket() = extendToPosition(ArmExtensionPosition.LOW_BASKET)

    fun extendFully() = extendToPosition(ArmExtensionPosition.MAX_UP)

    fun retract() = extendToPosition(ArmExtensionPosition.HOME)

    val isExtensionHome: Boolean
        get() = extensionHomeSensor?.isPressed == true

    val currentPosition: Int?
        get() = extensionMotorGroup?.currentPosition

    val extensionPositions: List<Int>
        get() = extensionMotorGroup?.getPositions() ?: listOf()

    companion object {
        const val EXTENSION_SPEED = 1.0
        const val ROTATION_LIMIT_POSITION = 1850
    }
}

enum class ArmExtensionPosition(val position: Int) {
    HOME(0),
    LOW_BASKET(1162),
    AUTON_PICKUP(2600),
    MAX_DOWN(5100),
    MAX_UP(6200)
}