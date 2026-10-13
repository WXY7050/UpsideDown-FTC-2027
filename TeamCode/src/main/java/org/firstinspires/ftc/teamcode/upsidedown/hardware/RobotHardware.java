package org.firstinspires.ftc.teamcode.upsidedown.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotHardware {
    // goBILDA Pinpoint 里程计
    public GoBildaPinpointDriver odo;

    // 吸取系统
    public DcMotorEx intake;

    // 发射系统
    public DcMotorEx shooter1;
    public DcMotorEx shooter2;
    public Servo block;

    // 距离传感器（球路检测，数字信号）
    public DigitalChannel disF;  // 最外侧
    public DigitalChannel disM;  // 中间
    public DigitalChannel disB;  // 最里面（靠近shooter）

    // LED灯（PWM信号控制）
    public Servo ledL;
    public Servo ledR;

    // 伺服位置常量
    public static final double BLOCK_CLOSE = 0.5;
    public static final double BLOCK_OPEN = 0.75;
    public static final double INTAKE_UP = 0.6;
    public static final double INTAKE_DOWN = 0.5;

    private HardwareMap hwMap;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // 初始化Pinpoint里程计
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        //odo.setOffsets(-84.0, -168.0, );
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        odo.resetPosAndIMU();

        // 初始化吸取系统
        intake = hwMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 初始化发射系统
        shooter1 = hwMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hwMap.get(DcMotorEx.class, "shooter2");
        block = hwMap.get(Servo.class, "block");
        shooter1.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.REVERSE);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // shooter1/shooter2由同步带刚性连接，转速由shooter1的码盘统一测量、
        // 由外部PID统一计算功率后镜像给两个电机，因此两者都用RUN_WITHOUT_ENCODER，
        // 避免电机内部再叠加一层基于编码器的闭环（shooter2甚至没有码盘接线）
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 初始化距离传感器
        disF = hwMap.get(DigitalChannel.class, "disF");
        disM = hwMap.get(DigitalChannel.class, "disM");
        disB = hwMap.get(DigitalChannel.class, "disB");
        disF.setMode(DigitalChannel.Mode.INPUT);
        disM.setMode(DigitalChannel.Mode.INPUT);
        disB.setMode(DigitalChannel.Mode.INPUT);

        // 初始化LED灯
        ledL = hwMap.get(Servo.class, "ledL");
        ledR = hwMap.get(Servo.class, "ledR");
    }

    public void updateOdo() {
        odo.update();
    }

    public void resetHeading() {
        odo.resetPosAndIMU();
    }

    public double getHeading() {
        return odo.getHeading(AngleUnit.RADIANS);
    }

    // shooter1是goBILDA 5203/5204系列无减速箱裸轴电机：编码器分辨率固定28 ticks/转（电机轴=输出轴，减速比1:1）。
    // DS配置界面里选的电机型号对应的ticksPerRev/maxRPM和实际减速比不一致，因此这里直接硬编码真实值，不依赖getMotorType()。
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double SHOOTER_MAX_RPM = 6000.0;

    // shooter1、shooter2用同步带刚性连接，转速统一由shooter1的码盘测量
    public double getShooterRPM() {
        double ticksPerSecond = shooter1.getVelocity();
        return ticksPerSecond / SHOOTER_TICKS_PER_REV * 60.0;
    }

    // shooter1满功率(power=1.0)时理论能达到的转速，用于PID前馈项
    public double getShooterMaxRPM() {
        return SHOOTER_MAX_RPM;
    }
}