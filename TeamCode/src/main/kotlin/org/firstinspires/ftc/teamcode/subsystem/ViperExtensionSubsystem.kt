package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.HardwareManager
import org.firstinspires.ftc.teamcode.subsystem.internal.MotorGroupSubsystem

class ViperExtensionSubsystem(hardware: HardwareManager) : MotorGroupSubsystem() {
    private val extensionMotorGroup = hardware.viperExtensionMotorGroup
    private val extensionHomeSensor = hardware.extensionHomeSensor

    init {
        extensionMotorGroup?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        extensionMotorGroup?.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun setExtensionMotorGroupPower(power: Double) {
        if (extensionMotorGroup == null) return

        if (power == 0.0) {
            extensionMotorGroup.power = 0.0
            return
        }

        if (extensionHomeSensor?.isPressed == true && power < 0) {
            extensionMotorGroup.setWithoutCorrection(0.0)
            return
        }

        val currentPosition = extensionMotorGroup.getCurrentPosition()
        if (currentPosition >= EXTENSION_MAX_POSITION && power > 0) {
            extensionMotorGroup.setWithoutCorrection(0.0)
        } else {
            extensionMotorGroup.setWithoutCorrection(getBoundedPower(power))
        }
    }

    fun extendToPosition(position: Int) {
        val safePosition = getBoundedPosition(position, EXTENSION_MIN_POSITION, EXTENSION_MAX_POSITION)

        extensionMotorGroup?.safelyGoToPosition(safePosition, EXTENSION_SPEED)
    }

    fun extendLowerBasket() = extendToPosition(EXTENSION_LOWER_BASKET_POSITION)

    fun extendFully() = extendToPosition(EXTENSION_MAX_POSITION)

    fun retract() = extendToPosition(EXTENSION_MIN_POSITION)

    val isExtensionHome: Boolean
        get() = extensionHomeSensor?.isPressed == true

    companion object {
        const val EXTENSION_SPEED = 1.0
        const val EXTENSION_MIN_POSITION = 0
        const val EXTENSION_PICKUP_POSITION = 500
        const val EXTENSION_LOWER_BASKET_POSITION = 1162
        const val EXTENSION_MAX_POSITION = 6100
    }
}