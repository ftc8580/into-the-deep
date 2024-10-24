package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.hardware.HardwareManager
import org.firstinspires.ftc.teamcode.util.MotorGroup

class ViperArmSubsystem(
    hardware: HardwareManager,
    private val telemetry: MultipleTelemetry? = null
) : SubsystemBase() {
    private val extensionMotorGroup = hardware.viperExtensionMotorGroup
    private val rotationMotorGroup = hardware.viperRotationMotorGroup

    init {
        extensionMotorGroup?.resetEncoder()
        extensionMotorGroup?.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        extensionMotorGroup?.setRunMode(Motor.RunMode.PositionControl)
        rotationMotorGroup?.positionCoefficient = 0.1
        rotationMotorGroup?.setPositionTolerance(5.0)

        rotationMotorGroup?.resetEncoder()
        rotationMotorGroup?.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        rotationMotorGroup?.setRunMode(Motor.RunMode.PositionControl)
        rotationMotorGroup?.positionCoefficient = 0.1
        rotationMotorGroup?.setPositionTolerance(5.0)
    }

    fun setMotorGroupsRawPower() {
        extensionMotorGroup?.setRunMode(Motor.RunMode.RawPower)
        rotationMotorGroup?.setRunMode(Motor.RunMode.RawPower)
    }

    fun setMotorGroupsPositionControl() {
        extensionMotorGroup?.setRunMode(Motor.RunMode.PositionControl)
        rotationMotorGroup?.setRunMode(Motor.RunMode.PositionControl)
    }

    fun correctExtensionGroupFollower() {
        extensionMotorGroup?.correctFollower(Motor.RunMode.RawPower)
    }

    fun correctRotationGroupFollower() {
        rotationMotorGroup?.correctFollower(Motor.RunMode.RawPower)
    }

    fun setExtensionMotorGroupPower(power: Double) {
        extensionMotorGroup?.set(getBoundedPower(power))
    }

    fun setRotationMotorGroupPower(power: Double) {
        rotationMotorGroup?.set(getBoundedPower(power))
    }

    fun getExtensionMotorGroupSpeed(): Double = extensionMotorGroup?.get() ?: 0.0

    fun getRotationMotorGroupSpeed(): Double = rotationMotorGroup?.get() ?: 0.0

    //Redefined to position instead of speed. MotorGroup only has function to get complete list of positions for all motors. First motor in list is "leader".
    //The leader is the first in list
    fun getExtensionMotorGroupPosition(): Double {
        val extensionMotorGroupPosList = extensionMotorGroup?.getPositions() ?: listOf()
        val extensionMotorGroupPosListFirst =  extensionMotorGroupPosList.first()
        return extensionMotorGroupPosListFirst
    }
    // The follower will be the second (last in this case) in the list
    fun getExtensionMotorGroupPositionLast(): Double {
        val extensionMotorGroupPosList = extensionMotorGroup?.getPositions() ?: listOf()
        val extensionMotorGroupPosListLast = extensionMotorGroupPosList.last()
        return extensionMotorGroupPosListLast
    }

    //Added for testing
    //TODO: Remove?
    fun getExtensionMotorGroupPositionList(): List<Double> {
        val extensionMotorGroupPosList = extensionMotorGroup?.getPositions() ?: listOf()
        return extensionMotorGroupPosList
    }

    // The leader is the first in list
    fun getRotationMotorGroupPosition(): Double {
        val rotationMotorGroupPosList = rotationMotorGroup?.getPositions() ?: listOf()
        val rotationMotorGroupPosListFirst = rotationMotorGroupPosList.first()
        return rotationMotorGroupPosListFirst
    }

    // The follower will be the second (last in this case) in the list
    fun getRotationMotorGroupPositionLast(): Double {
        val rotationMotorGroupPosList = rotationMotorGroup?.getPositions() ?: listOf()
        val rotationMotorGroupPosListLast = rotationMotorGroupPosList.last()
        return rotationMotorGroupPosListLast
    }

    //Added for testing
    //TODO: Remove?
    fun getRotationMotorGroupPositionList(): List<Double> {
        val rotationMotorGroupPosList = rotationMotorGroup?.getPositions() ?: listOf()
        return rotationMotorGroupPosList
    }

    fun extendToPosition(position: Int) {
        val safePosition = getBoundedPosition(position, EXTENSION_MIN_POSITION, EXTENSION_MAX_POSITION)

        safelyGoToPosition(extensionMotorGroup, safePosition, EXTENSION_SPEED)
    }

    fun rotateToPosition(position: Int) {
        val safePosition = getBoundedPosition(position, ROTATION_MIN_POSITION, ROTATION_MAX_POSITION)

        safelyGoToPosition(rotationMotorGroup, safePosition, ROTATION_SPEED)
    }

    fun extendLowerBasket() = extendToPosition(EXTENSION_LOWER_BASKET_POSITION)

    fun extendFully() = extendToPosition(EXTENSION_MAX_POSITION)

    fun retract() = extendToPosition(EXTENSION_MIN_POSITION)

    fun rotateHome() = rotateToPosition(ROTATION_MIN_POSITION)

    fun rotateTop() = rotateToPosition(ROTATION_MAX_POSITION)

    fun deliverTopBasket() {
        rotateTop()
        extendFully()
    }

    fun deliverLowerBasket() {
        rotateTop()
        extendLowerBasket()
    }

    fun home() {
        retract()
        rotateHome()
    }

    private fun safelyGoToPosition(
        motorGroup: MotorGroup?,
        targetPosition: Int,
        motorSpeed: Double
    ) = motorGroup?.let {
        it.setTargetPosition(targetPosition)
        it.set(motorSpeed)
    }

    private fun getBoundedPosition(position: Int, min: Int, max: Int): Int =
        Range.clip(position, min, max)

    private fun getBoundedPower(power: Double, min: Double = -1.0, max: Double = 1.0): Double =
        Range.clip(power, min, max)

    companion object {
        private const val EXTENSION_SPEED = 1.0
        private const val EXTENSION_MIN_POSITION = 0
        private const val EXTENSION_LOWER_BASKET_POSITION = 5
        private const val EXTENSION_MAX_POSITION = 10

        private const val ROTATION_SPEED = 0.1
        private const val ROTATION_MIN_POSITION = 0
        private const val ROTATION_MAX_POSITION = 10

        private const val EXTENSION_KP = 0.0
        private const val ROTATION_KP = 0.0
    }
}