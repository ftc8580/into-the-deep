package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.HardwareManager
import org.firstinspires.ftc.teamcode.subsystem.internal.MotorGroupSubsystem

class ViperExtensionSubsystem(hardware: HardwareManager) : MotorGroupSubsystem() {
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

        val currentPosition = extensionMotorGroup.getCurrentPosition()
        if (currentPosition >= ArmExtensionPosition.MAX_UP.position && power > 0) {
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

        // ROTATION value 1850 == max up position crosses plane below this
    }
}

enum class ArmExtensionPosition(val position: Int) {
    HOME(0),
    LOW_BASKET(1162),
    MAX_DOWN(5100),
    MAX_UP(6200)
}