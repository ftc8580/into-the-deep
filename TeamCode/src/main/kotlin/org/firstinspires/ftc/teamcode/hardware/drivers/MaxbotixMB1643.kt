package org.firstinspires.ftc.teamcode.hardware.drivers

import com.qualcomm.robotcore.hardware.AnalogInputController
import com.qualcomm.robotcore.hardware.AnalogSensor
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer
import com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.util.Range
import kotlin.math.min

@AnalogSensorType
@DeviceProperties(
    name = "MaxBotix MB1643",
    xmlTag = "MaxBotixMB1643",
    builtIn = false,
    description = "Ultrasonic Distance Sensor from MaxBotix"
)
class MaxbotixMB1643(
    private val analogInputController: AnalogInputController,
    private val physicalPort: Int
) : AnalogSensor, HardwareDevice {
    override fun readRawVoltage(): Double =
        analogInputController.getAnalogInputVoltage(physicalPort)

    override fun toString() = "MB1643 Distance: ${currentMillimeters}mm / ${currentInches}in"

    fun getRawMaxVoltage(): Double = analogInputController.maxAnalogInputVoltage

    fun getMaxVoltage(): Double {
        // The sensor itself is a 5.5v sensor, reporting analog values from 0v to 5.5v. However, depending
        // on the level conversion hardware that might be between us and the sensor, that may get shifted
        // to a different range. We'll assume that we only ever shift *down* in range, not up, so we
        // can take the min of the sensor's natural level and what the input controller can do.
        return min(VOLTAGE_MAX, analogInputController.maxAnalogInputVoltage)
    }

    fun getBoundedVoltage() = Range.clip(readRawVoltage(), VOLTAGE_MIN, getMaxVoltage())

    override fun getManufacturer() = Manufacturer.MaxBotix

    override fun getDeviceName(): String = "MB1643"

    override fun getConnectionInfo(): String = status()

    override fun getVersion(): Int = 1

    override fun resetDeviceConfigurationForOpMode() {}

    override fun close() {}

    fun status() =
        "Optical Distance Sensor, connected via device ${analogInputController.serialNumber}, port $physicalPort"

    val currentMillimeters: Double
        get() = Range.clip(
            ((getBoundedVoltage() / (getMaxVoltage() / 1024)) * 7.2) - 310,
            DISTANCE_MIN,
            DISTANCE_MAX
        )

    val currentInches: Double
        get() = currentMillimeters / 25.3

    companion object {
        private const val VOLTAGE_MIN = 0.0
        private const val VOLTAGE_MAX = 5.5
        private const val DISTANCE_MIN = 20.0
        private const val DISTANCE_MAX = 5000.0
    }
}