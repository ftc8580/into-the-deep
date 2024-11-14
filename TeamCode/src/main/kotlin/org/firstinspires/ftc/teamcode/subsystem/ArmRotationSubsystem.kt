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

        // TODO: Replace with encoder
//        if (rotationHomeSensor?.isPressed == true && power > 0) {
//            rotationMotorGroup.setWithoutCorrection(0.0)
//            return
//        }

        val currentPosition = -rotationMotorGroup.getPositions().first()
        if (currentPosition >= ROTATION_MAX_POSITION && power < 0) {
            rotationMotorGroup.setWithoutCorrection(0.0)
        } else {
            rotationMotorGroup.setWithoutCorrection(getBoundedPower(power))
        }
    }

    fun rotateToPosition(position: Int) {
        val safePosition = getBoundedPosition(position, ROTATION_MIN_POSITION, ROTATION_MAX_POSITION)

        rotationMotorGroup?.safelyGoToPosition(-safePosition, ROTATION_SPEED)
    }

    fun rotateHome() = rotateToPosition(ROTATION_MIN_POSITION)

    fun rotateTop() = rotateToPosition(ROTATION_MAX_POSITION)

    companion object {
        const val ROTATION_SPEED = 0.5
        const val ROTATION_MIN_POSITION = 0
        const val ROTATION_PICKUP_POSITION_GROUND = 700
        const val ROTATION_PICKUP_POSITION = 900
        const val ROTATION_DRIVE_POSITION = 1100
        const val ROTATION_AUTORUNG_POSITION = 2300
        const val ROTATION_MAX_POSITION = 4465
    }
}