package org.firstinspires.ftc.teamcode.hardware

import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.config.CDConfig
import org.firstinspires.ftc.teamcode.util.DeadWheelEncoder
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil
import org.firstinspires.ftc.teamcode.util.MotorGroup

class HardwareManager(private val config: CDConfig, hardware: HardwareMap) {
    lateinit var batteryVoltageSensor: VoltageSensor
    private lateinit var leftFrontMotor: DcMotorEx
    private lateinit var leftRearMotor: DcMotorEx
    private lateinit var rightRearMotor: DcMotorEx
    private lateinit var rightFrontMotor: DcMotorEx

    lateinit var driveMotors: List<DcMotorEx>

    // Dead wheels
    var leftEncoder: DeadWheelEncoder? = null
    var rightEncoder: DeadWheelEncoder? = null
    var rearEncoder: DeadWheelEncoder? = null

    // Sensors
    var extensionHomeSensor: TouchSensor? = null
    var rotationHomeSensor: TouchSensor? = null

    // Servos
    var intakeWheelServoRear: CRServo? = null
    var intakeWheelServoFront: CRServo? = null
    var intakeRotateServo: Servo? = null
    var gripperServo: Servo? = null

    // Accessory  motors
    var viperExtensionMotorLeft: Motor? = null
    var viperExtensionMotorRight: Motor? = null
    var viperRotationMotorLeft: Motor? = null
    var viperRotationMotorRight: Motor? = null
    var viperExtensionMotorGroup: MotorGroup? = null
    var viperRotationMotorGroup: MotorGroup? = null

    init {
        systemCheck(hardware)
        initializeBatteryVoltageSensor(hardware)
        initializeLynxModules(hardware)
        initializeWheelLocalizers(hardware)
        initializeDriveMotors(hardware)
        initializeSensors(hardware)
        initializeServos(hardware)
        initializeAccessoryMotors(hardware)
    }

    private fun systemCheck(hardware: HardwareMap)  {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardware)
    }

    private fun initializeBatteryVoltageSensor(hardware: HardwareMap) {
        batteryVoltageSensor = hardware.voltageSensor.iterator().next()
    }

    private fun initializeLynxModules(hardware: HardwareMap) {
        for (module in hardware.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
    }

    private fun initializeWheelLocalizers(hardware: HardwareMap) {
        val leftEncoderMotor = safelyGetHardware<DcMotorEx>(hardware, config.driveMotors.rightFront)
        val rightEncoderMotor = safelyGetHardware<DcMotorEx>(hardware, config.driveMotors.rightRear)
        val rearEncoderMotor = safelyGetHardware<DcMotorEx>(hardware, config.driveMotors.leftRear)

        // If any of the encoders are missing, don't initialize any of them
        if (leftEncoderMotor == null || rightEncoderMotor == null || rearEncoderMotor == null) return

        leftEncoder = DeadWheelEncoder(leftEncoderMotor)
        rightEncoder = DeadWheelEncoder(rightEncoderMotor)
        rearEncoder = DeadWheelEncoder(rearEncoderMotor)
    }

    private fun initializeDriveMotors(hardware: HardwareMap) {
        leftFrontMotor = hardware.get(DcMotorEx::class.java, config.driveMotors.leftFront)
        leftRearMotor = hardware.get(DcMotorEx::class.java, config.driveMotors.leftRear)
        rightRearMotor = hardware.get(DcMotorEx::class.java, config.driveMotors.rightRear)
        rightFrontMotor = hardware.get(DcMotorEx::class.java, config.driveMotors.rightFront)

        leftFrontMotor.direction = DcMotorSimple.Direction.REVERSE
        leftRearMotor.direction = DcMotorSimple.Direction.REVERSE
        rightRearMotor.direction = DcMotorSimple.Direction.FORWARD
        rightFrontMotor.direction = DcMotorSimple.Direction.FORWARD

        driveMotors = listOf(leftFrontMotor, leftRearMotor, rightRearMotor, rightFrontMotor)

        for (motor in driveMotors) {
            val motorConfigurationType = motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType

            // Set zero power behavior
            motor.zeroPowerBehavior = ZeroPowerBehavior.BRAKE

            // Run without encoder since we're using dead wheels
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    private fun initializeSensors(hardware: HardwareMap) {
        extensionHomeSensor = safelyGetHardware<TouchSensor>(hardware, "extensionHomeSensor")
        rotationHomeSensor = safelyGetHardware<TouchSensor>(hardware, "rotationHomeSensor")
    }

    private fun initializeServos(hardware: HardwareMap) {
        intakeWheelServoRear = safelyGetHardware<CRServo>(hardware, "intakeWheelServo")
        intakeWheelServoFront = safelyGetHardware<CRServo>(hardware, "intakeWheelServoFront")
        intakeRotateServo = safelyGetHardware<Servo>(hardware, "intakeRotateServo")
        gripperServo = safelyGetHardware<Servo>(hardware, "gripperServo")

        // Set ranges
        gripperServo?.scaleRange(0.0, 0.2)

        // Initialize servo positions
        // intakeRotateServo?.position = ActiveIntakeSubsystem.
        gripperServo?.position = 0.0
    }

    private fun initializeAccessoryMotors(hardware: HardwareMap) {
        viperExtensionMotorRight = safelyGetMotor(hardware, "viperExtensionRight", GoBILDA.RPM_117)
        viperExtensionMotorLeft = safelyGetMotor(hardware, "viperExtensionLeft", GoBILDA.RPM_117)
        viperRotationMotorRight = safelyGetMotor(hardware, "viperRotationRight", GoBILDA.RPM_43)
        viperRotationMotorLeft = safelyGetMotor(hardware, "viperRotationLeft", GoBILDA.RPM_43)

        // Sync sides
        viperExtensionMotorRight?.motor?.direction = DcMotorSimple.Direction.REVERSE
        viperRotationMotorRight?.motor?.direction = DcMotorSimple.Direction.REVERSE

        // Set encoder mode
        viperExtensionMotorRight?.motor?.mode = DcMotor.RunMode.RUN_USING_ENCODER
        viperExtensionMotorLeft?.motor?.mode = DcMotor.RunMode.RUN_USING_ENCODER
        viperRotationMotorRight?.motor?.mode = DcMotor.RunMode.RUN_USING_ENCODER
        viperRotationMotorLeft?.motor?.mode = DcMotor.RunMode.RUN_USING_ENCODER

        if (viperExtensionMotorRight != null && viperExtensionMotorLeft != null) {
            viperExtensionMotorGroup = MotorGroup(viperExtensionMotorRight!!, viperExtensionMotorLeft!!)
        }

        if (viperRotationMotorRight != null && viperRotationMotorLeft != null) {
            viperRotationMotorGroup = MotorGroup(viperRotationMotorRight!!, viperRotationMotorLeft!!)
        }
    }

    private inline fun <reified T> safelyGetHardware(hardware: HardwareMap, deviceName: String?): T? {
        if (deviceName.isNullOrBlank()) return null

        return try {
            hardware.get(T::class.java, deviceName)
        } catch (e: Exception) {
            println("Problem getting hardware $deviceName")
            null
        }
    }

    private fun safelyGetMotor(hardware: HardwareMap, deviceName: String, goBildaType: GoBILDA): Motor? =
        try {
            Motor(hardware, deviceName, goBildaType)
        } catch (e: Exception) {
            println("Problem getting motor $deviceName")
            null
        }

    val rawExternalHeading: Double
        get() = 0.0

    val externalHeadingVelocity: Double
        get() = 0.0

    fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
        leftFrontMotor.power = frontLeft
        leftRearMotor.power = rearLeft
        rightRearMotor.power = rearRight
        rightFrontMotor.power = frontRight
    }
}