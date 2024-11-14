package org.firstinspires.ftc.teamcode.config

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.drive.MecanumDrive

@Config
class MecanumDriveParams {
    companion object {
        // drive model parameters
        @JvmField var inPerTick: Double = 0.0019725
        @JvmField var lateralInPerTick: Double = 0.0012526365125091314
        @JvmField var trackWidthTicks: Double = 6930.281791018224

        // feedforward parameters (in tick units)
        @JvmField var kS: Double = 1.6138453046169712
        @JvmField var kV: Double = 0.00036
        @JvmField var kA: Double = 0.00005

        // path profile parameters (in inches)
        @JvmField var maxWheelVel: Double = 50.0
        @JvmField var minProfileAccel: Double = -30.0
        @JvmField var maxProfileAccel: Double = 50.0

        // turn profile parameters (in radians)
        @JvmField var maxAngVel: Double = Math.PI // shared with path
        @JvmField var maxAngAccel: Double = Math.PI

        // path controller gains
        @JvmField var axialGain: Double = 10.0
        @JvmField var lateralGain: Double = 15.0
        @JvmField var headingGain: Double = 15.0 // shared with turn

        @JvmField var axialVelGain: Double = 0.0
        @JvmField var lateralVelGain: Double = 0.0
        @JvmField var headingVelGain: Double = 0.0 // shared with turn
    }
}