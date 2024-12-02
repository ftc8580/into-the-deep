package org.firstinspires.ftc.teamcode.config

import com.acmerobotics.dashboard.config.Config

@Config
class LocalizerParams {
    companion object {
        @JvmField
        var par0YTicks: Double =
            -3137.0484377235553 // y position of the first parallel encoder (in tick units)
        @JvmField
        var par1YTicks: Double =
            3078.6380338343993 // y position of the second parallel encoder (in tick units)
        @JvmField
        var perpXTicks: Double =
            -3237.6280965219244 // x position of the perpendicular encoder (in tick units)
    }
}