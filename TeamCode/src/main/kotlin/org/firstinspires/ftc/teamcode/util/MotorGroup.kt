package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.math.abs

class MotorGroup(
    private val leader: DcMotorEx,
    private val follower: DcMotorEx
) : DcMotorEx {
    private val group: List<DcMotorEx> = listOf(leader, follower)
    private var isFollowerCorrected = true

    override fun getVelocity(): Double = leader.velocity

    override fun getVelocity(unit: AngleUnit?): Double = leader.getVelocity(unit)

    @Deprecated("Deprecated in Java")
    override fun setPIDCoefficients(mode: DcMotor.RunMode?, pidCoefficients: PIDCoefficients?) {
        group.forEach {
            it.setPIDCoefficients(mode, pidCoefficients)
        }
    }

    override fun setPIDFCoefficients(mode: DcMotor.RunMode?, pidfCoefficients: PIDFCoefficients?) {
        group.forEach {
            it.setPIDFCoefficients(mode, pidfCoefficients)
        }
    }

    override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) {
        group.forEach {
            it.setVelocityPIDFCoefficients(p, i, d, f)
        }
    }

    override fun setPositionPIDFCoefficients(p: Double) {
        group.forEach {
            it.setPositionPIDFCoefficients(p)
        }
    }

    @Deprecated("Deprecated in Java")
    override fun getPIDCoefficients(mode: DcMotor.RunMode?): PIDCoefficients {
        val output = leader.getPIDCoefficients(mode)
        follower.getPIDCoefficients(mode)
        return output
    }

    override fun getPIDFCoefficients(mode: DcMotor.RunMode?): PIDFCoefficients {
        val output = leader.getPIDFCoefficients(mode)
        follower.getPIDFCoefficients(mode)
        return output
    }

    override fun setTargetPositionTolerance(tolerance: Int) {
        group.forEach {
            it.targetPositionTolerance = tolerance
        }
    }

    override fun getTargetPositionTolerance(): Int = leader.targetPositionTolerance

    override fun getCurrent(unit: CurrentUnit?): Double = leader.getCurrent(unit)

    override fun getCurrentAlert(unit: CurrentUnit?): Double = leader.getCurrentAlert(unit)

    override fun setCurrentAlert(current: Double, unit: CurrentUnit?) {
        group.forEach { it.setCurrentAlert(current, unit) }
    }

    override fun isOverCurrent(): Boolean {
        return leader.isOverCurrent
    }

    fun getVelocities(): List<Double> = group.map { it.velocity }

    fun getPositions(): List<Int> = group.map { it.currentPosition }

    override fun getManufacturer(): HardwareDevice.Manufacturer = leader.manufacturer

    override fun getDeviceName(): String = leader.deviceName

    override fun getConnectionInfo(): String = leader.connectionInfo

    override fun getVersion(): Int = leader.version

    override fun resetDeviceConfigurationForOpMode() {
        group.forEach { it.resetDeviceConfigurationForOpMode() }
    }

    override fun close() {
        group.forEach { it.close() }
    }

    override fun setDirection(direction: DcMotorSimple.Direction?) {
        group.forEach { it.direction = direction }
    }

    fun setDirections(leaderDirection: DcMotorSimple.Direction, followerDirection: DcMotorSimple.Direction) {
        leader.direction = leaderDirection
        follower.direction = followerDirection
    }

    override fun getDirection(): DcMotorSimple.Direction = leader.direction

    fun getDirections(): List<DcMotorSimple.Direction> {
        return group.map { it.direction }
    }

    override fun setPower(power: Double) {
        group.forEach { it.power = power }
    }

    override fun getPower(): Double = leader.power

    override fun getMotorType(): MotorConfigurationType = leader.motorType

    override fun setMotorType(motorType: MotorConfigurationType?) {
        group.forEach { it.motorType = motorType }
    }

    override fun getController(): DcMotorController = leader.controller

    fun getControllers(): List<DcMotorController> {
        return group.map { it.controller }
    }

    override fun getPortNumber(): Int = leader.portNumber

    fun getPortNumbers(): List<Int> {
        return group.map { it.portNumber }
    }

    override fun setZeroPowerBehavior(behavior: DcMotor.ZeroPowerBehavior?) {
        behavior?.let {
            group.forEach { it.zeroPowerBehavior = behavior }
        }
    }

    override fun getZeroPowerBehavior(): DcMotor.ZeroPowerBehavior = leader.zeroPowerBehavior

    @Deprecated("Deprecated in Java")
    override fun setPowerFloat() {
        group.forEach { it.setPowerFloat() }
    }

    override fun getPowerFloat(): Boolean = leader.powerFloat

    override fun setTargetPosition(target: Int) {
        group.forEach { it.targetPosition = target }
    }

    override fun getTargetPosition(): Int = leader.targetPosition

    override fun isBusy(): Boolean = leader.isBusy

    override fun getCurrentPosition(): Int = leader.currentPosition

    fun getCurrentPositions(): List<Int> {
        return group.map { it.currentPosition }
    }

    override fun setMode(mode: DcMotor.RunMode?) {
        group.forEach { it.mode = mode }
    }

    override fun getMode(): DcMotor.RunMode = leader.mode

    override fun setMotorEnable() {
        group.forEach { it.setMotorEnable() }
    }

    override fun setMotorDisable() {
        group.forEach { it.setMotorDisable() }
    }

    override fun isMotorEnabled(): Boolean = leader.isMotorEnabled

    override fun setVelocity(angularRate: Double) {
        group.forEach { it.velocity = angularRate }
    }

    override fun setVelocity(angularRate: Double, unit: AngleUnit?) {
        group.forEach { it.setVelocity(angularRate, unit) }
    }

    fun correctFollower() {
        if (isFollowerCorrected) return

        val drift = followerDrift()

        if (abs(drift) > MAX_DRIFT) {
            val power = if (drift > 0) {
                -0.1
            } else {
                0.1
            }

            while (abs(followerDrift()) > MAX_DRIFT) {
                follower.power = power
            }
            follower.power = 0.0
        }

        isFollowerCorrected = true
    }

    private fun followerDrift(): Int {
        val leaderPosition = leader.currentPosition
        val followerPosition = follower.currentPosition

        return followerPosition - leaderPosition
    }

    companion object {
        private const val MAX_DRIFT = 20
    }
}