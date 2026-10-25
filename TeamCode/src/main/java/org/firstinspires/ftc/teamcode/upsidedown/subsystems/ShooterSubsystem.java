package org.firstinspires.ftc.teamcode.upsidedown.subsystems;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.upsidedown.RobotConstants;
import org.firstinspires.ftc.teamcode.upsidedown.hardware.RobotHardware;

/**
 * Shooter 子系统
 *
 * 功能：
 * - 双电机转速 PID 闭环控制
 * - 电压补偿
 * - RPM 监控
 * - Block 舵机控制
 */
public class ShooterSubsystem {

    private RobotHardware hardware;
    private VoltageSensor voltageSensor;

    // PID 参数（可在运行时调整）
    private double targetRPM;
    private double rpmTolerance;
    private double kF;
    private double kP;
    private double kI;
    private double kD;
    private double maxIntegralOutput;
    private double nominalVoltage;

    // PID 内部状态
    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTime = 0;

    // 当前状态
    private double currentPower = 0.0;
    private double currentRPM = 0.0;
    private double currentError = 0.0;

    // PID 各项输出（用于调试）
    private double fOutput = 0.0;
    private double pOutput = 0.0;
    private double iOutput = 0.0;
    private double dOutput = 0.0;

    public ShooterSubsystem(RobotHardware hardware, VoltageSensor voltageSensor) {
        this.hardware = hardware;
        this.voltageSensor = voltageSensor;

        // 从 Constants 加载默认参数
        this.targetRPM = RobotConstants.SHOOTER_TARGET_RPM;
        this.rpmTolerance = RobotConstants.SHOOTER_RPM_TOLERANCE;
        this.kF = RobotConstants.SHOOTER_KF;
        this.kP = RobotConstants.SHOOTER_KP;
        this.kI = RobotConstants.SHOOTER_KI;
        this.kD = RobotConstants.SHOOTER_KD;
        this.maxIntegralOutput = RobotConstants.SHOOTER_MAX_INTEGRAL_OUTPUT;
        this.nominalVoltage = RobotConstants.NOMINAL_VOLTAGE;

        resetPID();
    }

    /**
     * 更新 Shooter 控制循环（每个 loop() 调用一次）
     */
    public void update() {
        // 获取当前 RPM
        currentRPM = hardware.getShooterRPM();
        currentError = targetRPM - currentRPM;

        // 计算时间增量
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1_000_000_000.0;
        lastTime = currentTime;

        // 防止 dt 异常
        if (dt <= 0 || dt > 0.2) {
            dt = 0.02;
        }

        // Integral
        integral += currentError * dt;

        // Anti-Windup
        if (kI != 0.0) {
            double maxIntegral = maxIntegralOutput / Math.abs(kI);
            integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
        } else {
            integral = 0.0;
        }

        // Derivative
        double derivative = (currentError - lastError) / dt;
        lastError = currentError;

        // PIDF 各项输出
        fOutput = kF * targetRPM;
        pOutput = kP * currentError;
        iOutput = kI * integral;
        dOutput = kD * derivative;

        // Raw Power
        double rawPower = fOutput + pOutput + iOutput + dOutput;

        // 电压补偿
        double batteryVoltage = voltageSensor.getVoltage();
        if (batteryVoltage < 1.0) {
            batteryVoltage = nominalVoltage;
        }
        double voltageCompensation = nominalVoltage / batteryVoltage;

        // Final Power
        currentPower = rawPower * voltageCompensation;
        currentPower = Math.max(0.0, Math.min(1.0, currentPower));

        // 应用到电机
        hardware.shooter1.setPower(currentPower);
        hardware.shooter2.setPower(currentPower);
    }

    /** 停止 Shooter */
    public void stop() {
        hardware.shooter1.setPower(0.0);
        hardware.shooter2.setPower(0.0);
        resetPID();
    }

    /** Shooter 是否达到目标转速 */
    public boolean isReady() { return Math.abs(currentError) <= rpmTolerance; }

    /** 打开 Block 舵机 */
    public void openBlock() { hardware.block.setPosition(RobotConstants.BLOCK_OPEN); }

    /** 关闭 Block 舵机 */
    public void closeBlock() { hardware.block.setPosition(RobotConstants.BLOCK_CLOSE); }

    /**
     * 重置 PID 状态
     */
    public void resetPID() {
        integral = 0.0;
        lastError = 0.0;
        lastTime = System.nanoTime();
    }

    // =========================================================
    // Getters / Setters
    // =========================================================

    public double getTargetRPM() { return targetRPM; }
    public void setTargetRPM(double targetRPM) { this.targetRPM = targetRPM; }
    public double getCurrentRPM() { return currentRPM; }
    public double getError() { return currentError; }
    public double getCurrentPower() { return currentPower; }
    public double getRpmTolerance() { return rpmTolerance; }
    public void setRpmTolerance(double rpmTolerance) { this.rpmTolerance = rpmTolerance; }
    public double getkF() { return kF; }
    public void setkF(double kF) { this.kF = kF; }
    public double getkP() { return kP; }
    public void setkP(double kP) { this.kP = kP; }
    public double getkI() { return kI; }
    public void setkI(double kI) { this.kI = kI; }
    public double getkD() { return kD; }
    public void setkD(double kD) { this.kD = kD; }
    public double getNominalVoltage() { return nominalVoltage; }
    public void setNominalVoltage(double nominalVoltage) { this.nominalVoltage = nominalVoltage; }

    // 调试用
    public double getfOutput() { return fOutput; }
    public double getpOutput() { return pOutput; }
    public double getiOutput() { return iOutput; }
    public double getdOutput() { return dOutput; }
    public double getBatteryVoltage() { return voltageSensor.getVoltage(); }
}