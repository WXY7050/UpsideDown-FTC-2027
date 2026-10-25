package org.firstinspires.ftc.teamcode.upsidedown;

/**
 * UpsideDown 机器人统一常量配置
 *
 * 包含：
 * - Shooter PID 参数
 * - Servo 位置常量
 * - LED 颜色常量
 * - 传感器阈值
 * - 电压补偿参数
 */
public class RobotConstants {

    // =========================================================
    // Shooter 参数
    // =========================================================

    /** Shooter 默认目标转速 (RPM) */
    public static final double SHOOTER_TARGET_RPM = 3750.0;

    /** Shooter RPM 允许误差（第一球发射条件） */
    public static final double SHOOTER_RPM_TOLERANCE = 50;

    /** Shooter 电机编码器分辨率 (ticks/转) */
    public static final double SHOOTER_TICKS_PER_REV = 28.0;

    /** Shooter 电机最大转速 (RPM) */
    public static final double SHOOTER_MAX_RPM = 6000.0;


    // =========================================================
    // Shooter PIDF 参数
    // =========================================================

    /** Feed Forward 系数 */
    public static final double SHOOTER_KF = 0.00017020;

    /** 比例系数 */
    public static final double SHOOTER_KP = 0.000446;

    /** 积分系数 */
    public static final double SHOOTER_KI = 0.0;

    /** 微分系数 */
    public static final double SHOOTER_KD = 0.000001;

    /** I 项最大允许贡献的 motor power */
    public static final double SHOOTER_MAX_INTEGRAL_OUTPUT = 0.20;


    // =========================================================
    // 电压补偿
    // =========================================================

    /** 标准电压 (V) */
    public static final double NOMINAL_VOLTAGE = 12.9;


    // =========================================================
    // Intake / Feed 参数
    // =========================================================

    /** Intake 吸球功率 */
    public static final double INTAKE_POWER = 1.0;

    /** Intake 吐球功率 */
    public static final double OUTTAKE_POWER = -1.0;

    /** 发射时 Feed 速度 */
    public static final double FEED_POWER = 0.45;


    // =========================================================
    // Servo 位置常量
    // =========================================================

    /** Block 舵机 - 关闭位置 */
    public static final double BLOCK_CLOSE = 0.6;

    /** Block 舵机 - 打开位置 */
    public static final double BLOCK_OPEN = 0.8;

    /** Intake 舵机 - 抬起位置 */
    public static final double INTAKE_UP = 0.6;

    /** Intake 舵机 - 放下位置 */
    public static final double INTAKE_DOWN = 0.5;

    /** Flower 舵机 - 放下位置 */
    public static final double FLOWER_DOWN = 0.24;

    /** Flower 舵机 - 抬起位置 */
    public static final double FLOWER_UP = 0.57;


    // =========================================================
    // LED 颜色常量
    // =========================================================

    /** LED 深蓝色（满球提示） */
    public static final double LED_DEEP_BLUE = 0.615;

    /** LED 关闭 */
    public static final double LED_OFF = 0.0;

    /** LED 默认颜色 */
    public static final double LED_DEFAULT = 0.48;


    // =========================================================
    // 传感器
    // =========================================================

    /**
     * 满球判断：
     * disF && disM && disB
     *
     * 三个数字传感器：
     * disF = 最外侧
     * disM = 中间
     * disB = 最内侧（靠近 Shooter）
     */
}
