package org.firstinspires.ftc.teamcode.upsidedown.subsystems;

import org.firstinspires.ftc.teamcode.upsidedown.RobotConstants;
import org.firstinspires.ftc.teamcode.upsidedown.hardware.RobotHardware;

/**
 * Flower 子系统
 *
 * 功能：
 * - 控制 Flower 舵机的上升/下降
 */
public class FlowerSubsystem {

    private RobotHardware hardware;

    public FlowerSubsystem(RobotHardware hardware) {
        this.hardware = hardware;
    }

    /** Flower 抬起 */
    public void up() { hardware.flower.setPosition(RobotConstants.FLOWER_UP); }

    /** Flower 放下 */
    public void down() { hardware.flower.setPosition(RobotConstants.FLOWER_DOWN); }

    /** 设置 Flower 到指定位置 */
    public void setPosition(double position) { hardware.flower.setPosition(position); }

    /** 获取当前 Flower 位置 */
    public double getPosition() { return hardware.flower.getPosition(); }

    /** 判断 Flower 是否在上升位置 */
    public boolean isUp() { return Math.abs(hardware.flower.getPosition() - RobotConstants.FLOWER_UP) < 0.01; }

    /** 判断 Flower 是否在下降位置 */
    public boolean isDown() { return Math.abs(hardware.flower.getPosition() - RobotConstants.FLOWER_DOWN) < 0.01; }
}
