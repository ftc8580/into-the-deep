package org.firstinspires.ftc.teamcode.config

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection

object ImuParams {
    // IMU orientation
    // TODO: fill in these values based on
    //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
    var logoFacingDirection: RevHubOrientationOnRobot.LogoFacingDirection =
        RevHubOrientationOnRobot.LogoFacingDirection.UP
    var usbFacingDirection: UsbFacingDirection = UsbFacingDirection.BACKWARD
}