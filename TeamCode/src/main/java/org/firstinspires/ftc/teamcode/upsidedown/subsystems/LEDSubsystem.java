package org.firstinspires.ftc.teamcode.upsidedown.subsystems;

import org.firstinspires.ftc.teamcode.upsidedown.RobotConstants;
import org.firstinspires.ftc.teamcode.upsidedown.hardware.RobotHardware;

/**
 * LED 子系统
 *
 * 功能：
 * - 控制左右两侧 LED 灯的颜色
 * - 提供预设颜色模式
 */
public class LEDSubsystem {

    private RobotHardware hardware;

    public LEDSubsystem(RobotHardware hardware) {
        this.hardware = hardware;
    }

    /**
     * 设置两侧 LED 为相同颜色
     * @param color PWM 信号值 (0.0 - 1.0)
     */
    public void setColor(double color) {
        hardware.ledL.setPosition(color);
        hardware.ledR.setPosition(color);
    }

    /**
     * 分别设置左右 LED 颜色
     * @param leftColor 左侧 LED 颜色
     * @param rightColor 右侧 LED 颜色
     */
    public void setColor(double leftColor, double rightColor) {
        hardware.ledL.setPosition(leftColor);
        hardware.ledR.setPosition(rightColor);
    }

    /** 设置左侧 LED 颜色 */
    public void setLeftColor(double color) { hardware.ledL.setPosition(color); }

    /** 设置右侧 LED 颜色 */
    public void setRightColor(double color) { hardware.ledR.setPosition(color); }

    /** 关闭 LED */
    public void off() { setColor(RobotConstants.LED_OFF); }

    /** 显示深蓝色（满球提示） */
    public void showDeepBlue() { setColor(RobotConstants.LED_DEEP_BLUE); }

    /** 显示默认颜色 */
    public void showDefault() { setColor(RobotConstants.LED_DEFAULT); }

    /**
     * 根据球的状态自动设置 LED
     * @param ballsFull 是否满球
     */
    public void setByBallStatus(boolean ballsFull) {
        if (ballsFull) {
            showDeepBlue();
        } else {
            off();
        }
    }

    /**
     * LED 闪烁效果（需要在 loop 中持续调用）
     * @param intervalMs 闪烁间隔（毫秒）
     * @param color 闪烁颜色
     */
    public void blink(long intervalMs, double color) {
        long currentTime = System.currentTimeMillis();
        if ((currentTime / intervalMs) % 2 == 0) {
            setColor(color);
        } else {
            off();
        }
    }

    /** 获取左侧 LED 当前颜色 */
    public double getLeftColor() { return hardware.ledL.getPosition(); }

    /** 获取右侧 LED 当前颜色 */
    public double getRightColor() { return hardware.ledR.getPosition(); }
}