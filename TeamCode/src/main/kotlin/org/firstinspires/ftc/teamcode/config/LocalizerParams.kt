package org.firstinspires.ftc.teamcode.config

import com.acmerobotics.dashboard.config.Config

@Config
class LocalizerParams {
    companion object {
        @JvmField
        var par0YTicks: Double =
            -3091.9317284303575 // y position of the first parallel encoder (in tick units)
        @JvmField
        var par1YTicks: Double =
            3083.104672695253 // y position of the second parallel encoder (in tick units)
        @JvmField
        var perpXTicks: Double =
            -2884.4319874095117 // x position of the perpendicular encoder (in tick units)
    }
}