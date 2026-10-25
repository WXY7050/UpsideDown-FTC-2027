package org.firstinspires.ftc.teamcode.upsidedown.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.upsidedown.RobotConstants;
import org.firstinspires.ftc.teamcode.upsidedown.hardware.RobotHardware;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;


@Config
@Autonomous(
        name = "UpsideDown 红联盟右侧 Auto V3.13 Tune",
        group = "UpsideDown Auto"
)
public class UpsideDownRedRightAuto extends OpMode {


    // =========================================================
    // Hardware / Pedro
    // =========================================================

    private RobotHardware robot;
    private Follower follower;
    private VoltageSensor voltageSensor;


    // =========================================================
    // Dashboard 可调参数
    // =========================================================


    // ---------------------------------------------------------
    // 起点 / Preload 发射位
    // ---------------------------------------------------------

    public static double START_X = 58.0;
    public static double START_Y = 6.4175;
    public static double START_H = 90.0;


    // ---------------------------------------------------------
    // 第一条 Path 控制点
    // ---------------------------------------------------------

    public static double INTAKE_CONTROL_X = 58.0;
    public static double INTAKE_CONTROL_Y = 17.5;


    // ---------------------------------------------------------
    // 第一组四球起点
    // ---------------------------------------------------------

    public static double INTAKE_START_X = 19.85;
    public static double INTAKE_START_Y = 10.0;
    public static double INTAKE_START_H = 195.0;


    // ---------------------------------------------------------
    // 第一组四球终点
    // ---------------------------------------------------------

    public static double INTAKE_END_X = 9.40;
    public static double INTAKE_END_Y = 10.0;
    public static double INTAKE_END_H = 190.0;


    // ---------------------------------------------------------
    // 去第二发射位的控制点
    // ---------------------------------------------------------

    public static double SECOND_CONTROL_X = 18.81;
    public static double SECOND_CONTROL_Y = 36.25;


    // ---------------------------------------------------------
    // 第二发射位置
    // ---------------------------------------------------------

    public static double SECOND_SHOOT_X = 35.50;
    public static double SECOND_SHOOT_Y = 127.72;

    // 当前实测继续从 300° 调
    public static double SECOND_SHOOT_H = 300.0;

    // 第三组发射与第二组统一使用 300°
    public static double THIRD_SHOOT_H = 300.0;


    // ---------------------------------------------------------
    // 第二组吸球起点
    // ---------------------------------------------------------

    public static double SECOND_INTAKE_START_X = 22;
    public static double SECOND_INTAKE_START_Y = 48;
    public static double SECOND_INTAKE_START_H = 180.0;


    // ---------------------------------------------------------
    // 第二组吸球终点
    // ---------------------------------------------------------

    public static double SECOND_INTAKE_END_X = 14.30;
    public static double SECOND_INTAKE_END_Y = 48;
    public static double SECOND_INTAKE_END_H = 180.0;


    // =========================================================
    // Path Speed
    // =========================================================

    // Preload → 第一组四球
    public static double PATH_TO_INTAKE_SPEED = 1.00;

    // 第一组四球慢速吸取
    public static double INTAKE_PATH_SPEED = 0.5;

    // 第一组吸球目标时间 / 强制超时保护
    public static double FIRST_INTAKE_TARGET_TIME = 2.50;
    public static double FIRST_INTAKE_MAX_TIME = 2.80;

    // 第一组四球 → 第二发射位
    public static double RETURN_PATH_SPEED = 1.00;

    // 第二发射位 → 第二组吸球起点
    public static double PATH_TO_SECOND_INTAKE_SPEED = 1.00;

    // 第二组真正吸球速度
    public static double SECOND_INTAKE_PATH_SPEED = 0.5;

    // 第二组吸球目标时间：用于观察和调速
    public static double SECOND_INTAKE_TARGET_TIME = 1.00;

    // 第二组吸球最大允许时间：最终强制退出，避免卡死
    public static double SECOND_INTAKE_MAX_TIME = 1.30;

    // Flower 区物理极限保护：X 到这里就强制退出
    public static double SECOND_INTAKE_MIN_X = 14.30;

    // 第二组吸完后，保持 180° 倒车回吸球起点
    public static double SECOND_INTAKE_BACK_SPEED = 0.60;

    // 回到同一个发射位的速度
    public static double THIRD_RETURN_PATH_SPEED = 1.00;

    // 第二组吸取结束后连续回第三组发射位，出发 0.5 秒后抬 Flower
    public static double THIRD_RETURN_FLOWER_UP_DELAY = 0.50;

    // 第三次发射完成后前往第三组吸取起点
    public static double AFTER_THIRD_X = 47.0;
    public static double AFTER_THIRD_Y = 113.0;
    public static double AFTER_THIRD_H = 90.0;
    public static double AFTER_THIRD_PATH_SPEED = 0.5;

    // 第三组吸取终点：Intake 开启后以 50% Path Speed 向 +Y 走
    public static double THIRD_INTAKE_END_X = 47.0;
    public static double THIRD_INTAKE_END_Y = 128.0;
    public static double THIRD_INTAKE_END_H = 90.0;
    public static double THIRD_INTAKE_PATH_SPEED = 0.50;

    // 第三组吸取至少 2 秒，2.30 秒为最终超时保护
    public static double THIRD_INTAKE_TARGET_TIME = 2.00;
    public static double THIRD_INTAKE_MAX_TIME = 2.30;

    // 第三组 Flower 区 Y 方向机械极限保护
    public static double THIRD_INTAKE_MAX_Y = 130.0;

    // 第三组吸取完成后的中间点
    public static double POST_THIRD_X = 17.5;
    public static double POST_THIRD_Y = 57.0;
    public static double POST_THIRD_H = 90.0;

    // 第四组发射点
    public static double FINAL_X = 40.0;
    public static double FINAL_Y = 10.0;
    public static double FINAL_H = 71.0;
    public static double FINAL_PATH_SPEED = 1.00;

    // 连续曲线运行到 Y <= 50 后开始计时，0.5 秒后边走边抬 Flower
    public static double FINAL_FLOWER_TRIGGER_Y = 50.0;
    public static double FINAL_FLOWER_UP_DELAY = 0.50;

    // Flower 舵机动作等待时间
    public static double FLOWER_DOWN_WAIT_TIME = 0.30;
    public static double FLOWER_UP_WAIT_TIME = 0.30;


    // =========================================================
    // Shooter
    // =========================================================

    // 四轮发射独立参数，方便 Dashboard 单独调试
    public static double PRELOAD_TARGET_RPM = 3750.0;
    public static double PRELOAD_FEED_POWER = 0.30;

    public static double SECOND_TARGET_RPM = 3825.0;
    public static double SECOND_FEED_POWER = 0.35;

    public static double THIRD_TARGET_RPM = 3825.0;
    public static double THIRD_FEED_POWER = 0.35;

    public static double FOURTH_TARGET_RPM = 3825.0;
    public static double FOURTH_FEED_POWER = 0.35;

    public static double RPM_TOLERANCE = 50.0;
    public static double RPM_READY_CONFIRM_TIME = 0.15;


    // =========================================================
    // Fire Time
    // =========================================================

    public static double PRELOAD_FIRE_TIME = 1.80;

    public static double SECOND_FIRE_TIME = 1.40;

    // Feed 降到 0.35 后给第4球更多进入时间
    public static double THIRD_FIRE_TIME = 1.80;

    public static double FOURTH_FIRE_TIME = 1.80;


    // =========================================================
    // Intake
    // =========================================================

    public static double INTAKE_POWER = 1.0;


    // =========================================================
    // 第二轮 Shooter 提前起速
    // =========================================================

    public static double SECOND_SHOOTER_SPINUP_DISTANCE = 25.0;

    public static double THIRD_SHOOTER_SPINUP_DISTANCE = 25.0;

    public static double FOURTH_SHOOTER_SPINUP_DISTANCE = 25.0;

    // 第四组发射完成后的停靠点
    public static double PARK_X = 10.0;
    public static double PARK_Y = 92.0;
    public static double PARK_H = 90.0;
    public static double PARK_PATH_SPEED = 1.00;


    // =========================================================
    // Shooter PID
    // =========================================================

    public static double kF = 0.00017020;
    public static double kP = 0.000446;
    public static double kI = 0.0;
    public static double kD = 0.000001;

    public static double MAX_INTEGRAL_OUTPUT = 0.20;

    public static double NOMINAL_VOLTAGE = 12.9;


    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastPIDTime = 0;


    // =========================================================
    // Auto State
    // =========================================================

    private enum AutoState {

        // Preload
        SPIN_UP,
        FIRE_PRELOAD,

        // 第一组四球
        MOVE_TO_INTAKE,
        INTAKE_SWEEP,

        // 第二次发射
        RETURN_TO_SECOND_SHOOT,
        SECOND_SPIN_UP,
        SECOND_FIRE,

        // 第二组吸球
        MOVE_TO_SECOND_INTAKE,
        LOWER_FLOWER,
        SECOND_INTAKE_SWEEP,

        // 回同一个发射位，第三次发射
        RETURN_TO_THIRD_SHOOT,
        THIRD_SPIN_UP,
        THIRD_FIRE,

        // 第三次发射后：第三组 Flower 吸取 + 最终路径
        MOVE_AFTER_THIRD_FIRE,
        LOWER_THIRD_FLOWER,
        THIRD_INTAKE_SWEEP,
        MOVE_TO_FINAL_POINT,

        // 第四组发射 + 停靠
        FOURTH_SPIN_UP,
        FOURTH_FIRE,
        MOVE_TO_PARK,

        DONE
    }


    private AutoState state =
            AutoState.SPIN_UP;


    // =========================================================
    // Timers
    // =========================================================

    private long rpmReadyStartTime = -1;
    private long fireStartTime = -1;

    private long flowerWaitStartTime = -1;

    private long firstIntakeStartTime = -1;
    private double lastFirstIntakeTime = 0.0;

    private long secondIntakeStartTime = -1;
    private double lastSecondIntakeTime = 0.0;

    private long thirdIntakeStartTime = -1;
    private double lastThirdIntakeTime = 0.0;

    private long finalFlowerTriggerTime = -1;
    private long thirdReturnStartTime = -1;


    // =========================================================
    // Dynamic Poses
    // =========================================================

    private Pose getStartPose() { return new Pose(START_X, START_Y, Math.toRadians(START_H)); }
    private Pose getIntakeControlPose() { return new Pose(INTAKE_CONTROL_X, INTAKE_CONTROL_Y, 0.0); }
    private Pose getIntakeStartPose() { return new Pose(INTAKE_START_X, INTAKE_START_Y, Math.toRadians(INTAKE_START_H)); }
    private Pose getIntakeEndPose() { return new Pose(INTAKE_END_X, INTAKE_END_Y, Math.toRadians(INTAKE_END_H)); }
    private Pose getSecondControlPose() { return new Pose(SECOND_CONTROL_X, SECOND_CONTROL_Y, 0.0); }
    private Pose getSecondShootPose() { return new Pose(SECOND_SHOOT_X, SECOND_SHOOT_Y, Math.toRadians(SECOND_SHOOT_H)); }
    private Pose getThirdShootPose() { return new Pose(SECOND_SHOOT_X, SECOND_SHOOT_Y, Math.toRadians(THIRD_SHOOT_H)); }
    private Pose getAfterThirdPose() { return new Pose(AFTER_THIRD_X, AFTER_THIRD_Y, Math.toRadians(AFTER_THIRD_H)); }
    private Pose getThirdIntakeEndPose() { return new Pose(THIRD_INTAKE_END_X, THIRD_INTAKE_END_Y, Math.toRadians(THIRD_INTAKE_END_H)); }
    private Pose getPostThirdPose() { return new Pose(POST_THIRD_X, POST_THIRD_Y, Math.toRadians(POST_THIRD_H)); }
    private Pose getFinalPose() { return new Pose(FINAL_X, FINAL_Y, Math.toRadians(FINAL_H)); }
    private Pose getParkPose() { return new Pose(PARK_X, PARK_Y, Math.toRadians(PARK_H)); }
    private Pose getSecondIntakeStartPose() { return new Pose(SECOND_INTAKE_START_X, SECOND_INTAKE_START_Y, Math.toRadians(SECOND_INTAKE_START_H)); }
    private Pose getSecondIntakeEndPose() { return new Pose(SECOND_INTAKE_END_X, SECOND_INTAKE_END_Y, Math.toRadians(SECOND_INTAKE_END_H)); }


    // =========================================================
    // Paths
    // =========================================================


    // ---------------------------------------------------------
    // Preload 发射位 → 第一组四球起点
    // ---------------------------------------------------------

    private Path pathToIntakeStart() {
        Pose start = getStartPose();
        Pose control = getIntakeControlPose();
        Pose end = getIntakeStartPose();

        return curve(start, control, end).linear(start, end).with(Constants.foresightConfig.maxPathSpeed.at(PATH_TO_INTAKE_SPEED));
    }


    // ---------------------------------------------------------
    // 第一组四球慢速吸取
    // ---------------------------------------------------------

    private Path intakeSweepPath() {
        Pose start = follower.pose();
        Pose end = getIntakeEndPose();

        return line(start, end).linear(start, end).with(Constants.foresightConfig.maxPathSpeed.at(INTAKE_PATH_SPEED));
    }


    // ---------------------------------------------------------
    // 第一组四球终点 → 第二发射位
    // ---------------------------------------------------------

    private Path pathToSecondShoot() {
        Pose start = follower.pose();
        Pose control = getSecondControlPose();
        Pose end = getSecondShootPose();

        return curve(start, control, end).linear(start, end).with(Constants.foresightConfig.maxPathSpeed.at(RETURN_PATH_SPEED));
    }


    // ---------------------------------------------------------
    // 第二发射位 → 第二组吸球起点
    //
    // 第一版先直线测试
    // ---------------------------------------------------------

    private Path pathToSecondIntakeStart() {
        Pose start = getSecondShootPose();
        Pose end = getSecondIntakeStartPose();

        return line(start, end).linear(start, end).with(Constants.foresightConfig.maxPathSpeed.at(PATH_TO_SECOND_INTAKE_SPEED));
    }


    // ---------------------------------------------------------
    // 第二组慢速吸球
    // ---------------------------------------------------------

    private Path secondIntakeSweepPath() {
        Pose start = getSecondIntakeStartPose();
        Pose end = getSecondIntakeEndPose();

        return line(start, end).linear(start, end).with(Constants.foresightConfig.maxPathSpeed.at(SECOND_INTAKE_PATH_SPEED));
    }


    // ---------------------------------------------------------
    // 第二组吸球结束 → 第三组发射位
    // 当前真实 Pose → control (22,48,180°) → 第三组发射点
    // 一条连续 Bezier Curve，不在 (22,48) 停车。
    // ---------------------------------------------------------

    private Path pathToThirdShootContinuous() {
        Pose start = follower.pose();
        Pose control = getSecondIntakeStartPose();
        Pose end = getThirdShootPose();

        return curve(start, control, end).linear(start, end).with(Constants.foresightConfig.maxPathSpeed.at(THIRD_RETURN_PATH_SPEED));
    }


    // ---------------------------------------------------------
    // 第三次发射位 → 新测试点
    // (35.5, 127.72, 304°) → (45.5, 108.3, 90°)
    // 先用直线 + Heading 插值测试能否顺利转到 90°
    // ---------------------------------------------------------

    private Path pathAfterThirdFire() {
        Pose start = follower.pose();
        Pose end = getAfterThirdPose();

        return line(start, end).linear(start, end).with(Constants.foresightConfig.maxPathSpeed.at(AFTER_THIRD_PATH_SPEED));
    }


    // ---------------------------------------------------------
    // (47,113,90°) → 第三组吸取终点
    // Flower DOWN + Intake ON，满速吸取
    // ---------------------------------------------------------

    private Path thirdIntakeSweepPath() {
        Pose start = getAfterThirdPose();
        Pose end = getThirdIntakeEndPose();

        return line(start, end).linear(start, end).with(Constants.foresightConfig.maxPathSpeed.at(THIRD_INTAKE_PATH_SPEED));
    }


    // ---------------------------------------------------------
    // 第三组吸取结束 → 第四组发射点
    // 当前真实 Pose → control (17.5,57) → end (40,7,71°)
    // 一条连续 Bezier Curve，不在中间点停车。
    // ---------------------------------------------------------

    private Path pathToFourthShoot() {
        Pose start = follower.pose();
        Pose control = new Pose(POST_THIRD_X, POST_THIRD_Y, Math.toRadians(POST_THIRD_H));
        Pose end = getFinalPose();

        return curve(start, control, end).linear(start, end).with(Constants.foresightConfig.maxPathSpeed.at(FINAL_PATH_SPEED));
    }


    // ---------------------------------------------------------
    // 第四组发射完成 → 停靠点 (10,87,90°)
    // ---------------------------------------------------------

    private Path pathToPark() {
        Pose start = follower.pose();
        Pose end = getParkPose();

        return line(start, end).linear(start, end).with(Constants.foresightConfig.maxPathSpeed.at(PARK_PATH_SPEED));
    }


    // =========================================================
    // INIT
    // =========================================================

    @Override
    public void init() {
        robot = new RobotHardware();
        robot.init(hardwareMap);

        follower = Constants.create(hardwareMap);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // 初始机构状态
        robot.intake.setPower(0.0);
        robot.shooter1.setPower(0.0);
        robot.shooter2.setPower(0.0);
        robot.block.setPosition(RobotConstants.BLOCK_CLOSE);
        robot.flower.setPosition(RobotConstants.FLOWER_UP);

        telemetry.addLine("UpsideDown RED RIGHT Auto V3.13 Tune");
        telemetry.addData("Start", "(%.2f, %.2f, %.1f°)", START_X, START_Y, START_H);
        telemetry.addData("Second Shoot", "(%.2f, %.2f, %.1f°)", SECOND_SHOOT_X, SECOND_SHOOT_Y, SECOND_SHOOT_H);
        telemetry.addData("Second Intake Start", "(%.2f, %.2f, %.1f°)", SECOND_INTAKE_START_X, SECOND_INTAKE_START_Y, SECOND_INTAKE_START_H);
        telemetry.addData("Second Intake End", "(%.2f, %.2f, %.1f°)", SECOND_INTAKE_END_X, SECOND_INTAKE_END_Y, SECOND_INTAKE_END_H);
        telemetry.update();
    }


    // =========================================================
    // INIT LOOP
    // =========================================================

    @Override
    public void init_loop() {
        telemetry.addLine("========== AUTO TUNE ==========");
        telemetry.addData("Start", "(%.2f, %.2f, %.1f°)", START_X, START_Y, START_H);
        telemetry.addData("Intake 1 Start", "(%.2f, %.2f, %.1f°)", INTAKE_START_X, INTAKE_START_Y, INTAKE_START_H);
        telemetry.addData("Intake 1 End", "(%.2f, %.2f, %.1f°)", INTAKE_END_X, INTAKE_END_Y, INTAKE_END_H);
        telemetry.addData("Second Shoot", "(%.2f, %.2f, %.1f°)", SECOND_SHOOT_X, SECOND_SHOOT_Y, SECOND_SHOOT_H);
        telemetry.addData("Intake 2 Start", "(%.2f, %.2f, %.1f°)", SECOND_INTAKE_START_X, SECOND_INTAKE_START_Y, SECOND_INTAKE_START_H);
        telemetry.addData("Intake 2 End", "(%.2f, %.2f, %.1f°)", SECOND_INTAKE_END_X, SECOND_INTAKE_END_Y, SECOND_INTAKE_END_H);
        telemetry.addData("Intake 1 Speed", "%.0f%%", INTAKE_PATH_SPEED * 100.0);

        double firstIntakeTimeNow = state == AutoState.INTAKE_SWEEP ? (System.nanoTime() - firstIntakeStartTime) / 1_000_000_000.0 : lastFirstIntakeTime;
        telemetry.addData("Intake 1 Time", "%.2f s / target %.2f / max %.2f", firstIntakeTimeNow, FIRST_INTAKE_TARGET_TIME, FIRST_INTAKE_MAX_TIME);
        telemetry.addData("Intake 2 Speed", "%.0f%%", SECOND_INTAKE_PATH_SPEED * 100.0);
        telemetry.update();
    }


    // =========================================================
    // START
    // =========================================================

    @Override
    public void start() {
        follower.setPose(getStartPose());
        resetPID();
        state = AutoState.SPIN_UP;
        rpmReadyStartTime = -1;
        fireStartTime = -1;
        robot.intake.setPower(0.0);
        robot.block.setPosition(RobotConstants.BLOCK_CLOSE);
    }


    // =========================================================
    // LOOP
    // =========================================================

    @Override
    public void loop() {


        // =====================================================
        // Pedro
        // =====================================================
        follower.update();
        Pose pose = follower.pose();
        double robotX = pose.x();
        double robotY = pose.y();
        double robotHeadingDeg = normalize360(Math.toDegrees(pose.heading()));

        // =====================================================
        // 第二发射点距离
        // =====================================================
        Pose secondShootPose = getSecondShootPose();
        double distanceToSecondShoot = Math.hypot(secondShootPose.x() - robotX, secondShootPose.y() - robotY);

        Pose fourthShootPose = getFinalPose();
        double distanceToFourthShoot = Math.hypot(fourthShootPose.x() - robotX, fourthShootPose.y() - robotY);

        // =====================================================
        // Shooter RPM
        // =====================================================
        double currentRPM = robot.getShooterRPM();


        // 当前轮次使用独立目标 RPM。
        double activeTargetRPM;

        switch (state) {
            case SPIN_UP:
            case FIRE_PRELOAD:
                activeTargetRPM = PRELOAD_TARGET_RPM;
                break;

            case RETURN_TO_SECOND_SHOOT:
            case SECOND_SPIN_UP:
            case SECOND_FIRE:
                activeTargetRPM = SECOND_TARGET_RPM;
                break;

            case RETURN_TO_THIRD_SHOOT:
            case THIRD_SPIN_UP:
            case THIRD_FIRE:
                activeTargetRPM = THIRD_TARGET_RPM;
                break;

            case MOVE_TO_FINAL_POINT:
            case FOURTH_SPIN_UP:
            case FOURTH_FIRE:
                activeTargetRPM = FOURTH_TARGET_RPM;
                break;

            default:
                activeTargetRPM = 0.0;
                break;
        }


        double error = activeTargetRPM - currentRPM;

        // =====================================================
        // Shooter PID
        // =====================================================
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastPIDTime) / 1_000_000_000.0;
        lastPIDTime = currentTime;

        if (dt <= 0.0 || dt > 0.2) { dt = 0.02; }

        integral += error * dt;

        if (kI != 0.0) {
            double maxIntegral = MAX_INTEGRAL_OUTPUT / Math.abs(kI);
            integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
        } else {
            integral = 0.0;
        }

        double derivative = (error - lastError) / dt;
        lastError = error;

        double fOutput = kF * activeTargetRPM;
        double pOutput = kP * error;
        double iOutput = kI * integral;
        double dOutput = kD * derivative;
        double rawShooterPower = fOutput + pOutput + iOutput + dOutput;

        // =====================================================
        // Voltage Compensation
        // =====================================================
        double batteryVoltage = voltageSensor.getVoltage();
        if (batteryVoltage < 1.0) { batteryVoltage = NOMINAL_VOLTAGE; }
        double voltageCompensation = NOMINAL_VOLTAGE / batteryVoltage;
        double shooterPower = rawShooterPower * voltageCompensation;
        shooterPower = Math.max(0.0, Math.min(1.0, shooterPower));


        // =====================================================
        // Shooter Enable
        // =====================================================
        boolean shooterEnabled = state == AutoState.SPIN_UP
                || state == AutoState.FIRE_PRELOAD
                || state == AutoState.SECOND_SPIN_UP
                || state == AutoState.SECOND_FIRE
                || state == AutoState.THIRD_SPIN_UP
                || state == AutoState.THIRD_FIRE
                || state == AutoState.FOURTH_SPIN_UP
                || state == AutoState.FOURTH_FIRE
                || (state == AutoState.RETURN_TO_SECOND_SHOOT && distanceToSecondShoot <= SECOND_SHOOTER_SPINUP_DISTANCE)
                || (state == AutoState.RETURN_TO_THIRD_SHOOT && distanceToSecondShoot <= THIRD_SHOOTER_SPINUP_DISTANCE)
                || (state == AutoState.MOVE_TO_FINAL_POINT && distanceToFourthShoot <= FOURTH_SHOOTER_SPINUP_DISTANCE);

        if (shooterEnabled) {
            robot.shooter1.setPower(shooterPower);
            robot.shooter2.setPower(shooterPower);
        } else {
            robot.shooter1.setPower(0.0);
            robot.shooter2.setPower(0.0);
        }

        // =====================================================
        // 三颗球路传感器
        // =====================================================
        boolean ballAtF = robot.disF.getState();
        boolean ballAtM = robot.disM.getState();
        boolean ballAtB = robot.disB.getState();
        boolean ballsFull = ballAtF && ballAtM && ballAtB;

        // =====================================================
        // RPM Ready
        // =====================================================
        boolean rpmInTolerance = Math.abs(error) <= RPM_TOLERANCE;
        double rpmReadyTime = 0.0;

        if (shooterEnabled && rpmInTolerance) {
            if (rpmReadyStartTime < 0) { rpmReadyStartTime = System.nanoTime(); }
            rpmReadyTime = (System.nanoTime() - rpmReadyStartTime) / 1_000_000_000.0;
        } else {
            rpmReadyStartTime = -1;
        }

        boolean shooterReady = rpmReadyTime >= RPM_READY_CONFIRM_TIME;


        // =====================================================
        // STATE MACHINE
        // =====================================================
        switch (state) {

            // -------------------------------------------------
            // 1. Preload 起速
            // -------------------------------------------------
            case SPIN_UP:
                robot.intake.setPower(0.0);
                robot.block.setPosition(RobotConstants.BLOCK_CLOSE);

                if (shooterReady) {
                    fireStartTime = System.nanoTime();
                    state = AutoState.FIRE_PRELOAD;
                }
                break;

            // -------------------------------------------------
            // 2. Preload 发射
            // -------------------------------------------------
            case FIRE_PRELOAD:
                robot.block.setPosition(RobotConstants.BLOCK_OPEN);
                robot.intake.setPower(PRELOAD_FEED_POWER);

                double preloadFireTime = (System.nanoTime() - fireStartTime) / 1_000_000_000.0;

                if (preloadFireTime >= PRELOAD_FIRE_TIME) {
                    robot.intake.setPower(0.0);
                    robot.block.setPosition(RobotConstants.BLOCK_CLOSE);
                    follower.follow(pathToIntakeStart());
                    state = AutoState.MOVE_TO_INTAKE;
                }
                break;

            // -------------------------------------------------
            // 3. 去第一组四球
            // -------------------------------------------------
            case MOVE_TO_INTAKE:
                robot.intake.setPower(0.0);
                robot.block.setPosition(RobotConstants.BLOCK_CLOSE);

                if (!follower.isBusy()) {
                    robot.intake.setPower(INTAKE_POWER);
                    firstIntakeStartTime = System.nanoTime();
                    follower.follow(intakeSweepPath());
                    state = AutoState.INTAKE_SWEEP;
                }
                break;

            // -------------------------------------------------
            // 4. 第一组慢速吸球
            // -------------------------------------------------
            case INTAKE_SWEEP:
                robot.intake.setPower(INTAKE_POWER);
                robot.block.setPosition(RobotConstants.BLOCK_CLOSE);

                double firstIntakeTime = (System.nanoTime() - firstIntakeStartTime) / 1_000_000_000.0;
                boolean firstIntakePathFinished = !follower.isBusy();
                boolean firstMinIntakeTimeReached = firstIntakeTime >= FIRST_INTAKE_TARGET_TIME;
                boolean firstIntakeTimedOut = firstIntakeTime >= FIRST_INTAKE_MAX_TIME;
                boolean firstNormalExitReady = firstMinIntakeTimeReached && firstIntakePathFinished;

                if (firstNormalExitReady || firstIntakeTimedOut) {
                    lastFirstIntakeTime = firstIntakeTime;
                    follower.follow(pathToSecondShoot());
                    resetPID();
                    rpmReadyStartTime = -1;
                    state = AutoState.RETURN_TO_SECOND_SHOOT;
                }
                break;

            // -------------------------------------------------
            // 5. 去第二发射位
            // -------------------------------------------------
            case RETURN_TO_SECOND_SHOOT:
                if (ballsFull) {
                    robot.intake.setPower(0.0);
                } else {
                    robot.intake.setPower(INTAKE_POWER);
                }
                robot.block.setPosition(RobotConstants.BLOCK_CLOSE);

                if (!follower.isBusy()) {
                    robot.intake.setPower(0.0);
                    rpmReadyStartTime = -1;
                    state = AutoState.SECOND_SPIN_UP;
                }
                break;

            // -------------------------------------------------
            // 6. 第二发射位等 RPM
            // -------------------------------------------------
            case SECOND_SPIN_UP:
                robot.intake.setPower(0.0);
                robot.block.setPosition(RobotConstants.BLOCK_CLOSE);

                if (shooterReady) {
                    fireStartTime = System.nanoTime();
                    state = AutoState.SECOND_FIRE;
                }
                break;

            // -------------------------------------------------
            // 7. 第二组发射
            // -------------------------------------------------
            case SECOND_FIRE:
                robot.block.setPosition(RobotConstants.BLOCK_OPEN);
                robot.intake.setPower(SECOND_FEED_POWER);

                double secondFireTime = (System.nanoTime() - fireStartTime) / 1_000_000_000.0;

                if (secondFireTime >= SECOND_FIRE_TIME) {
                    robot.intake.setPower(0.0);
                    robot.block.setPosition(RobotConstants.BLOCK_CLOSE);
                    follower.follow(pathToSecondIntakeStart());
                    state = AutoState.MOVE_TO_SECOND_INTAKE;
                }
                break;


            // -------------------------------------------------
            // 8. 去第二组吸球起点
            // -------------------------------------------------
            case MOVE_TO_SECOND_INTAKE:
                robot.intake.setPower(0.0);
                robot.block.setPosition(RobotConstants.BLOCK_CLOSE);
                robot.flower.setPosition(RobotConstants.FLOWER_UP);

                if (!follower.isBusy()) {
                    robot.flower.setPosition(RobotConstants.FLOWER_DOWN);
                    flowerWaitStartTime = System.nanoTime();
                    state = AutoState.LOWER_FLOWER;
                }
                break;

            // -------------------------------------------------
            // 9. 等 Flower DOWN 到位
            // -------------------------------------------------
            case LOWER_FLOWER:
                robot.flower.setPosition(RobotConstants.FLOWER_DOWN);
                robot.intake.setPower(0.0);
                robot.block.setPosition(RobotConstants.BLOCK_CLOSE);

                double flowerDownWait = (System.nanoTime() - flowerWaitStartTime) / 1_000_000_000.0;

                if (flowerDownWait >= FLOWER_DOWN_WAIT_TIME) {
                    robot.intake.setPower(INTAKE_POWER);
                    secondIntakeStartTime = System.nanoTime();
                    follower.follow(secondIntakeSweepPath());
                    state = AutoState.SECOND_INTAKE_SWEEP;
                }
                break;

            // -------------------------------------------------
            // 10. 第二组慢速吸球
            // -------------------------------------------------
            case SECOND_INTAKE_SWEEP:
                robot.flower.setPosition(RobotConstants.FLOWER_DOWN);
                robot.intake.setPower(INTAKE_POWER);
                robot.block.setPosition(RobotConstants.BLOCK_CLOSE);

                double secondIntakeTime = (System.nanoTime() - secondIntakeStartTime) / 1_000_000_000.0;
                boolean intakePathFinished = !follower.isBusy();
                boolean reachedXLimit = robotX <= SECOND_INTAKE_MIN_X;
                boolean minIntakeTimeReached = secondIntakeTime >= SECOND_INTAKE_TARGET_TIME;
                boolean intakeTimedOut = secondIntakeTime >= SECOND_INTAKE_MAX_TIME;
                boolean normalExitReady = minIntakeTimeReached && (intakePathFinished || reachedXLimit);

                if (normalExitReady || intakeTimedOut) {
                    lastSecondIntakeTime = secondIntakeTime;
                    thirdReturnStartTime = System.nanoTime();
                    resetPID();
                    rpmReadyStartTime = -1;
                    follower.follow(pathToThirdShootContinuous());
                    state = AutoState.RETURN_TO_THIRD_SHOOT;
                }
                break;

            // -------------------------------------------------
            // 11. 连续回第三组发射位
            // -------------------------------------------------
            case RETURN_TO_THIRD_SHOOT:
                robot.block.setPosition(RobotConstants.BLOCK_CLOSE);

                double thirdReturnTime = (System.nanoTime() - thirdReturnStartTime) / 1_000_000_000.0;

                if (thirdReturnTime >= THIRD_RETURN_FLOWER_UP_DELAY) {
                    robot.flower.setPosition(RobotConstants.FLOWER_UP);
                    robot.intake.setPower(0.0);
                } else {
                    robot.flower.setPosition(RobotConstants.FLOWER_DOWN);
                    if (ballsFull) {
                        robot.intake.setPower(0.0);
                    } else {
                        robot.intake.setPower(INTAKE_POWER);
                    }
                }

                if (!follower.isBusy()) {
                    robot.flower.setPosition(RobotConstants.FLOWER_UP);
                    robot.intake.setPower(0.0);
                    rpmReadyStartTime = -1;
                    state = AutoState.THIRD_SPIN_UP;
                }
                break;

            // -------------------------------------------------
            // 14. 第三次发射前等 RPM Ready
            // -------------------------------------------------
            case THIRD_SPIN_UP:
                robot.flower.setPosition(RobotConstants.FLOWER_UP);
                robot.intake.setPower(0.0);
                robot.block.setPosition(RobotConstants.BLOCK_CLOSE);

                if (shooterReady) {
                    fireStartTime = System.nanoTime();
                    state = AutoState.THIRD_FIRE;
                }
                break;

            // -------------------------------------------------
            // 15. 第三次发射
            // -------------------------------------------------
            case THIRD_FIRE:
                robot.flower.setPosition(RobotConstants.FLOWER_UP);
                robot.block.setPosition(RobotConstants.BLOCK_OPEN);
                robot.intake.setPower(THIRD_FEED_POWER);

                double thirdFireTime = (System.nanoTime() - fireStartTime) / 1_000_000_000.0;

                if (thirdFireTime >= THIRD_FIRE_TIME) {
                    robot.intake.setPower(0.0);
                    robot.block.setPosition(RobotConstants.BLOCK_CLOSE);
                    follower.follow(pathAfterThirdFire());
                    state = AutoState.MOVE_AFTER_THIRD_FIRE;
                }
                break;


            // -------------------------------------------------
            // 16. 第三次发射后前往第三组吸取起点
            // -------------------------------------------------
            case MOVE_AFTER_THIRD_FIRE:
                robot.flower.setPosition(RobotConstants.FLOWER_UP);
                robot.intake.setPower(0.0);
                robot.block.setPosition(RobotConstants.BLOCK_CLOSE);

                if (!follower.isBusy()) {
                    robot.flower.setPosition(RobotConstants.FLOWER_DOWN);
                    flowerWaitStartTime = System.nanoTime();
                    state = AutoState.LOWER_THIRD_FLOWER;
                }
                break;

            // -------------------------------------------------
            // 17. 等第三组 Flower DOWN 到位
            // -------------------------------------------------
            case LOWER_THIRD_FLOWER:
                robot.flower.setPosition(RobotConstants.FLOWER_DOWN);
                robot.intake.setPower(0.0);
                robot.block.setPosition(RobotConstants.BLOCK_CLOSE);

                double thirdFlowerDownWait = (System.nanoTime() - flowerWaitStartTime) / 1_000_000_000.0;

                if (thirdFlowerDownWait >= FLOWER_DOWN_WAIT_TIME) {
                    robot.intake.setPower(INTAKE_POWER);
                    thirdIntakeStartTime = System.nanoTime();
                    follower.follow(thirdIntakeSweepPath());
                    state = AutoState.THIRD_INTAKE_SWEEP;
                }
                break;

            // -------------------------------------------------
            // 18. 第三组满速吸取
            // -------------------------------------------------
            case THIRD_INTAKE_SWEEP:
                robot.flower.setPosition(RobotConstants.FLOWER_DOWN);
                robot.intake.setPower(INTAKE_POWER);
                robot.block.setPosition(RobotConstants.BLOCK_CLOSE);

                double thirdIntakeTime = (System.nanoTime() - thirdIntakeStartTime) / 1_000_000_000.0;
                boolean thirdIntakePathFinished = !follower.isBusy();
                boolean reachedThirdYLimit = robotY >= THIRD_INTAKE_MAX_Y;
                boolean thirdMinIntakeTimeReached = thirdIntakeTime >= THIRD_INTAKE_TARGET_TIME;
                boolean thirdIntakeTimedOut = thirdIntakeTime >= THIRD_INTAKE_MAX_TIME;
                boolean thirdNormalExitReady = thirdMinIntakeTimeReached && (thirdIntakePathFinished || reachedThirdYLimit);

                if (thirdNormalExitReady || thirdIntakeTimedOut) {
                    lastThirdIntakeTime = thirdIntakeTime;
                    robot.intake.setPower(0.0);
                    robot.flower.setPosition(RobotConstants.FLOWER_DOWN);
                    finalFlowerTriggerTime = -1;
                    resetPID();
                    rpmReadyStartTime = -1;
                    follower.follow(pathToFourthShoot());
                    state = AutoState.MOVE_TO_FINAL_POINT;
                }
                break;

            // -------------------------------------------------
            // 19. 第三组吸取结束 → 第四组发射点
            // -------------------------------------------------
            case MOVE_TO_FINAL_POINT:
                robot.intake.setPower(0.0);
                robot.block.setPosition(RobotConstants.BLOCK_CLOSE);

                if (robotY <= FINAL_FLOWER_TRIGGER_Y && finalFlowerTriggerTime < 0) {
                    finalFlowerTriggerTime = System.nanoTime();
                }

                if (finalFlowerTriggerTime >= 0) {
                    double flowerUpDelayTime = (System.nanoTime() - finalFlowerTriggerTime) / 1_000_000_000.0;
                    if (flowerUpDelayTime >= FINAL_FLOWER_UP_DELAY) {
                        robot.flower.setPosition(RobotConstants.FLOWER_UP);
                    } else {
                        robot.flower.setPosition(RobotConstants.FLOWER_DOWN);
                    }
                } else {
                    robot.flower.setPosition(RobotConstants.FLOWER_DOWN);
                }

                if (!follower.isBusy()) {
                    robot.flower.setPosition(RobotConstants.FLOWER_UP);
                    rpmReadyStartTime = -1;
                    state = AutoState.FOURTH_SPIN_UP;
                }
                break;

            // -------------------------------------------------
            // 20. 第四组发射前等待 RPM Ready
            // -------------------------------------------------
            case FOURTH_SPIN_UP:
                robot.flower.setPosition(RobotConstants.FLOWER_UP);
                robot.intake.setPower(0.0);
                robot.block.setPosition(RobotConstants.BLOCK_CLOSE);

                if (shooterReady) {
                    fireStartTime = System.nanoTime();
                    state = AutoState.FOURTH_FIRE;
                }
                break;

            // -------------------------------------------------
            // 21. 第四组四球发射
            // -------------------------------------------------
            case FOURTH_FIRE:
                robot.flower.setPosition(RobotConstants.FLOWER_UP);
                robot.block.setPosition(RobotConstants.BLOCK_OPEN);
                robot.intake.setPower(FOURTH_FEED_POWER);

                double fourthFireTime = (System.nanoTime() - fireStartTime) / 1_000_000_000.0;

                if (fourthFireTime >= FOURTH_FIRE_TIME) {
                    robot.intake.setPower(0.0);
                    robot.block.setPosition(RobotConstants.BLOCK_CLOSE);
                    follower.follow(pathToPark());
                    state = AutoState.MOVE_TO_PARK;
                }
                break;

            // -------------------------------------------------
            // 22. 发射完成后停靠
            // -------------------------------------------------
            case MOVE_TO_PARK:
                robot.flower.setPosition(RobotConstants.FLOWER_UP);
                robot.intake.setPower(0.0);
                robot.block.setPosition(RobotConstants.BLOCK_CLOSE);

                if (!follower.isBusy()) {
                    state = AutoState.DONE;
                }
                break;

            // -------------------------------------------------
            // 23. DONE
            // -------------------------------------------------
            case DONE:
                robot.flower.setPosition(RobotConstants.FLOWER_UP);
                robot.intake.setPower(0.0);
                robot.block.setPosition(RobotConstants.BLOCK_CLOSE);
                break;
        }


        // =====================================================
        // Telemetry
        // =====================================================
        telemetry.addLine("========== RED RIGHT AUTO V3.13 ==========");
        telemetry.addData("State", state);
        telemetry.addLine("");
        telemetry.addData("Robot", "(%.2f, %.2f, %.1f°)", robotX, robotY, robotHeadingDeg);
        telemetry.addData("Follower Busy", follower.isBusy() ? "YES" : "NO");
        telemetry.addLine("");
        telemetry.addData("Second Shoot", "(%.2f, %.2f, %.1f°)", SECOND_SHOOT_X, SECOND_SHOOT_Y, SECOND_SHOOT_H);
        telemetry.addData("2nd Intake Start", "(%.2f, %.2f, %.1f°)", SECOND_INTAKE_START_X, SECOND_INTAKE_START_Y, SECOND_INTAKE_START_H);
        telemetry.addData("2nd Intake End", "(%.2f, %.2f, %.1f°)", SECOND_INTAKE_END_X, SECOND_INTAKE_END_Y, SECOND_INTAKE_END_H);
        telemetry.addLine("");
        telemetry.addData("Intake 1 Speed", "%.0f%%", INTAKE_PATH_SPEED * 100.0);

        double firstIntakeTimeNow = state == AutoState.INTAKE_SWEEP ? (System.nanoTime() - firstIntakeStartTime) / 1_000_000_000.0 : lastFirstIntakeTime;
        telemetry.addData("Intake 1 Time", "%.2f s / target %.2f / max %.2f", firstIntakeTimeNow, FIRST_INTAKE_TARGET_TIME, FIRST_INTAKE_MAX_TIME);
        telemetry.addData("Intake 2 Speed", "%.0f%%", SECOND_INTAKE_PATH_SPEED * 100.0);

        double secondIntakeDisplayTime = lastSecondIntakeTime;
        if (state == AutoState.SECOND_INTAKE_SWEEP && secondIntakeStartTime > 0) {
            secondIntakeDisplayTime = (System.nanoTime() - secondIntakeStartTime) / 1_000_000_000.0;
        }

        telemetry.addData("Intake 2 Time", "%.2f s / target %.2f / max %.2f", secondIntakeDisplayTime, SECOND_INTAKE_TARGET_TIME, SECOND_INTAKE_MAX_TIME);
        telemetry.addData("Intake X Limit", "X %.2f / min %.2f / %s", robotX, SECOND_INTAKE_MIN_X, robotX <= SECOND_INTAKE_MIN_X ? "REACHED" : "OK");
        telemetry.addData("Back Out Speed", "%.0f%%", SECOND_INTAKE_BACK_SPEED * 100.0);
        telemetry.addData("Third Return Speed", "%.0f%%", THIRD_RETURN_PATH_SPEED * 100.0);
        telemetry.addData("After Third Target", "(%.2f, %.2f, %.1f°)", AFTER_THIRD_X, AFTER_THIRD_Y, AFTER_THIRD_H);
        telemetry.addData("After Third Speed", "%.0f%%", AFTER_THIRD_PATH_SPEED * 100.0);

        double thirdIntakeDisplayTime = lastThirdIntakeTime;
        if (state == AutoState.THIRD_INTAKE_SWEEP && thirdIntakeStartTime > 0) {
            thirdIntakeDisplayTime = (System.nanoTime() - thirdIntakeStartTime) / 1_000_000_000.0;
        }

        telemetry.addData("Intake 3 Time", "%.2f s / target %.2f / max %.2f", thirdIntakeDisplayTime, THIRD_INTAKE_TARGET_TIME, THIRD_INTAKE_MAX_TIME);
        telemetry.addData("Intake 3 Y Limit", "Y %.2f / max %.2f / %s", robotY, THIRD_INTAKE_MAX_Y, robotY >= THIRD_INTAKE_MAX_Y ? "REACHED" : "OK");
        telemetry.addData("Post Third", "(%.2f, %.2f, %.1f°)", POST_THIRD_X, POST_THIRD_Y, POST_THIRD_H);
        telemetry.addData("Final Target", "(%.2f, %.2f, %.1f°)", FINAL_X, FINAL_Y, FINAL_H);
        telemetry.addData("Park Target", "(%.2f, %.2f, %.1f°)", PARK_X, PARK_Y, PARK_H);
        telemetry.addLine("");
        telemetry.addData("Active Target RPM", "%.0f", activeTargetRPM);
        telemetry.addData("Actual RPM", "%.0f", currentRPM);
        telemetry.addData("RPM Error", "%.0f", error);
        telemetry.addData("RPM Ready", shooterReady ? "YES" : "NO");
        telemetry.addData("Shooter Enabled", shooterEnabled ? "YES" : "NO");
        telemetry.addLine("");
        telemetry.addData("disF", ballAtF ? "BALL" : "EMPTY");
        telemetry.addData("disM", ballAtM ? "BALL" : "EMPTY");
        telemetry.addData("disB", ballAtB ? "BALL" : "EMPTY");
        telemetry.addData("Balls Full", ballsFull ? "YES" : "NO");
        telemetry.addLine("");
        telemetry.addData("Flower", "%.3f", robot.flower.getPosition());
        telemetry.addData("Battery", "%.2f V", batteryVoltage);
        telemetry.update();
    }


    // =========================================================
    // Reset PID
    // =========================================================

    private void resetPID() {
        integral = 0.0;
        lastError = 0.0;
        lastPIDTime = System.nanoTime();
    }

    private double normalize360(double angle) {
        angle %= 360.0;
        if (angle < 0.0) { angle += 360.0; }
        return angle;
    }


    // =========================================================
    // STOP
    // =========================================================

    @Override
    public void stop() {
        robot.intake.setPower(0.0);
        robot.shooter1.setPower(0.0);
        robot.shooter2.setPower(0.0);
        robot.block.setPosition(RobotConstants.BLOCK_CLOSE);
    }
}

