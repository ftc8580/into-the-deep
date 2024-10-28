package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.ColorSensor
import java.util.Locale
import kotlin.math.pow
import kotlin.math.sqrt

class RevColor(private val colorSensor: ColorSensor) {
    private val red: Int
        get() = colorSensor.red()

    private val green: Int
        get() = colorSensor.green()

    private val blue: Int
        get() = colorSensor.blue()

    private val total: Int
        get() = red + green + blue

    private val rgb: DoubleArray
        get() {
            val arr = DoubleArray(3)
            arr[0] = red.toDouble()
            arr[1] = green.toDouble()
            arr[2] = blue.toDouble()

            return arr
        }

    private val normalizedRgb: DoubleArray
        get() {
            val arr = DoubleArray(3)
            val originalArr = rgb

            var total = 0.0
            for (i in originalArr) total += i

            for (i in 0..2) {
                arr[i] = originalArr[i] / total
            }

            return arr
        }

    private fun arrayError(arr1: DoubleArray, arr2: DoubleArray): Double {
        var total = 0.0

        for (i in arr1.indices) {
            total += (arr1[i] - arr2[i]).pow(2.0)
        }

        return sqrt(total)
    }

    val isBlack: Boolean
        get() = colorSensor.alpha() < BLACK_ALPHA_VALUE

    val alpha: Int
        get() = colorSensor.alpha()

    val isRed: Boolean
        get() = colorSensor.red() > RED_THRESHOLD

    val isBlue: Boolean
        get() = colorSensor.blue() > BLUE_THRESHOLD

    private val yellowError: Double
        get() = arrayError(normalizedRgb, YELLOW_CONSTANTS)

    private val whiteError: Double
        get() = arrayError(normalizedRgb, WHITE_CONSTANTS)

    val isYellow: Boolean
        get() = yellowError < YELLOW_THRESHOLD

    val isWhite: Boolean
        get() = whiteError < WHITE_THRESHOLD && total > WHITE_TOTAL_COUNT

    fun normalizedValues(): String {
        val red = colorSensor.red().toDouble()
        val green = colorSensor.green().toDouble()
        val blue = colorSensor.blue().toDouble()

        val total = red + green + blue
        return String.format(Locale.US, "RGB: %.2f %.2f %.2f", red / total, green / total, blue / total)
    }

    // turn on the lights
    fun enableLED(ledMode: Boolean) {
        colorSensor.enableLed(ledMode)
    }

    fun withinColorRange(): Boolean {
        return isYellow || isWhite
    }

    companion object {
        // Base Color Threshold
        private const val RED_THRESHOLD: Double = 550.0
        private const val BLUE_THRESHOLD: Double = 550.0

        // Complex Color Thresholds (not red, green, or blue)
        private const val YELLOW_RED_VALUE: Double = 0.419
        private const val YELLOW_GREEN_VALUE: Double = 0.3675
        private const val YELLOW_BLUE_VALUE: Double = 0.2116
        private val YELLOW_CONSTANTS: DoubleArray = doubleArrayOf(YELLOW_RED_VALUE, YELLOW_GREEN_VALUE, YELLOW_BLUE_VALUE)
        private const val YELLOW_THRESHOLD: Double = 0.12

        private const val WHITE_RED_VALUE: Double = 0.30125
        private const val WHITE_GREEN_VALUE: Double = 0.37125
        private const val WHITE_BLUE_VALUE: Double = 0.325
        private val WHITE_CONSTANTS: DoubleArray = doubleArrayOf(WHITE_RED_VALUE, WHITE_GREEN_VALUE, WHITE_BLUE_VALUE)
        private const val WHITE_THRESHOLD: Double = 0.01
        private const val WHITE_TOTAL_COUNT: Double = 800.0

        // check for black with the alpha value
        private const val BLACK_ALPHA_VALUE: Double = 325.0 //Test value
    }
}