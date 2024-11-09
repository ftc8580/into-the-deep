package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d

object Drawing {
    fun drawRobot(c: Canvas, t: Pose2d) {
        val ROBOT_RADIUS = 9.0

        c.setStrokeWidth(1)
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS)

        val halfv: Vector2d = t.heading.vec().times(0.5 * ROBOT_RADIUS)
        val p1: Vector2d = t.position.plus(halfv)
        val p2: Vector2d = p1.plus(halfv)
        c.strokeLine(p1.x, p1.y, p2.x, p2.y)
    }
}