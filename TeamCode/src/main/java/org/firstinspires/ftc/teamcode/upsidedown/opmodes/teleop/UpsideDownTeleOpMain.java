package org.firstinspires.ftc.teamcode.upsidedown.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.pedropathing.follower.Follower;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.upsidedown.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.upsidedown.subsystems.DriveSubsystem;


@TeleOp(name = "UpsideDown 手动", group = "UpsideDown")
public class UpsideDownTeleOpMain extends OpMode {

    private Follower follower;
    private DriveSubsystem drive;
    private RobotHardware robot;

    private VoltageSensor voltageSensor;


    // =========================================================
    // 底盘
    // =========================================================

    private boolean fieldCentric = true;
    private boolean lastFieldCentricButton = false;


    // =========================================================
    // Shooter 参数
    // =========================================================

    // 目标转速
    public static double TARGET_RPM = 3775;

    // 第一球允许发射的 RPM 误差
    public static double RPM_TOLERANCE = 35;

    // 发射时 Feed 速度
    public static final double FEED_POWER = 0.45;


    // =========================================================
    // Shooter PIDF
    // =========================================================

    /*
     * Shooter 控制公式：
     *
     * rawPower =
     *      kF * TARGET_RPM
     *      + kP * error
     *      + kI * integral
     *      + kD * derivative
     *
     * finalPower =
     *      rawPower
     *      * NOMINAL_VOLTAGE
     *      / batteryVoltage
     */


    // Feed Forward
    public static double kF = 0.00017020;


    // PID
    public static double kP = 0.000446;

    public static double kI = 0.0;

    public static double kD = 0.000001;


    // I 项最大允许贡献的 motor power
    public static double MAX_INTEGRAL_OUTPUT = 0.20;


    // =========================================================
    // 电压补偿
    // =========================================================

    /*
     * 当前标准电压
     */

    public static final double NOMINAL_VOLTAGE = 12.9;


    // =========================================================
    // 满球 / LED 参数
    // =========================================================

    public static double BALL_DETECT_DISTANCE = 50.0;

    public static double LED_DEEP_BLUE = 0.615;


    // =========================================================
    // PID 内部变量
    // =========================================================

    private double integral = 0.0;

    private double lastError = 0.0;

    private long lastTime = 0;


    // =========================================================
    // Feed 状态
    // =========================================================

    /*
     * false：
     * 当前这一轮发射还没有达到过目标 RPM
     *
     * true：
     * 已经达到过一次目标 RPM
     * 本次右扳机一直按住期间
     * 持续 Feed，不再检测 RPM
     */

    private boolean feedUnlocked = false;


    // =========================================================
    // Init
    // =========================================================

    @Override
    public void init() {

        // Pedro Pathing
        follower = Constants.create(hardwareMap);


        // 底盘
        drive = new DriveSubsystem(hardwareMap);


        // Robot Hardware
        robot = new RobotHardware();

        robot.init(hardwareMap);


        // =====================================================
        // 电池电压传感器
        // =====================================================

        voltageSensor =
                hardwareMap.voltageSensor
                        .iterator()
                        .next();


        // 默认关闭 block
        robot.block.setPosition(
                RobotHardware.BLOCK_CLOSE
        );


        // 默认 LED
        robot.ledL.setPosition(
                0.48
        );

        robot.ledR.setPosition(
                0.48
        );


        telemetry.addData(
                "状态",
                "初始化完成"
        );

        telemetry.addData(
                "",
                "按下 Start 开始"
        );

        telemetry.update();
    }


    // =========================================================
    // Start
    // =========================================================

    @Override
    public void start() {

        resetPID();

        feedUnlocked = false;
    }


    // =========================================================
    // Loop
    // =========================================================

    @Override
    public void loop() {


        // =========================================================
        // Odometry
        // =========================================================

        robot.updateOdo();

        // follower.update();


        // =========================================================
        // 满球检测
        // =========================================================

        boolean ballAtF =
                robot.disF.getState();

        boolean ballAtM =
                robot.disM.getState();

        boolean ballAtB =
                robot.disB.getState();


        boolean ballsFull =
                ballAtF
                        && ballAtM
                        && ballAtB;


        // =========================================================
        // LED
        // =========================================================

        if (ballsFull) {

            robot.ledL.setPosition(
                    LED_DEEP_BLUE
            );

            robot.ledR.setPosition(
                    LED_DEEP_BLUE
            );

        } else {

            robot.ledL.setPosition(
                    0.00
            );

            robot.ledR.setPosition(
                    0.00
            );
        }


        // =========================================================
        // 底盘控制
        // =========================================================

        double forward =
                -gamepad1.left_stick_y;

        double strafe =
                gamepad1.left_stick_x;

        double rotate =
                gamepad1.right_stick_x;


        // Back：
        // Field Centric / Robot Centric 切换

        if (
                gamepad1.back
                        && !lastFieldCentricButton
        ) {

            fieldCentric =
                    !fieldCentric;
        }


        lastFieldCentricButton =
                gamepad1.back;


        if (fieldCentric) {

            drive.driveFieldCentric(
                    forward,
                    strafe,
                    rotate,
                    robot.getHeading()
            );

        } else {

            drive.driveRobotCentric(
                    forward,
                    strafe,
                    rotate
            );
        }


        // =========================================================
        // Shooter RPM
        // =========================================================

        double currentRPM =
                robot.getShooterRPM();


        double error =
                TARGET_RPM
                        - currentRPM;


        // =========================================================
        // PID 时间
        // =========================================================

        long currentTime =
                System.nanoTime();


        double dt =
                (currentTime - lastTime)
                        / 1_000_000_000.0;


        lastTime =
                currentTime;


        // 防止 dt 异常

        if (
                dt <= 0
                        || dt > 0.2
        ) {

            dt =
                    0.02;
        }


        // =========================================================
        // Integral
        // =========================================================

        integral +=
                error * dt;


        /*
         * Anti-Windup
         */

        if (
                kI != 0.0
        ) {

            double maxIntegral =
                    MAX_INTEGRAL_OUTPUT
                            / Math.abs(
                            kI
                    );


            integral =
                    Math.max(
                            -maxIntegral,
                            Math.min(
                                    maxIntegral,
                                    integral
                            )
                    );

        } else {

            integral =
                    0.0;
        }


        // =========================================================
        // Derivative
        // =========================================================

        double derivative =
                (error - lastError)
                        / dt;


        lastError =
                error;


        // =========================================================
        // PIDF 各项输出
        // =========================================================

        double fOutput =
                kF
                        * TARGET_RPM;


        double pOutput =
                kP
                        * error;


        double iOutput =
                kI
                        * integral;


        double dOutput =
                kD
                        * derivative;


        // =========================================================
        // PID Raw Power
        // =========================================================

        double rawShooterPower =
                fOutput
                        + pOutput
                        + iOutput
                        + dOutput;


        // =========================================================
        // 当前电池电压
        // =========================================================

        double batteryVoltage =
                voltageSensor.getVoltage();


        /*
         * 如果出现异常读数，
         * 不进行额外补偿。
         */

        if (
                batteryVoltage < 1.0
        ) {

            batteryVoltage =
                    NOMINAL_VOLTAGE;
        }


        // =========================================================
        // 电压补偿
        // =========================================================

        double voltageCompensation =
                NOMINAL_VOLTAGE
                        / batteryVoltage;


        // =========================================================
        // Final Shooter Power
        // =========================================================

        double shooterPower =
                rawShooterPower
                        * voltageCompensation;


        // Motor Power 限制 0 ~ 1

        shooterPower =
                Math.max(
                        0.0,
                        Math.min(
                                1.0,
                                shooterPower
                        )
                );


        // =========================================================
        // Shooter 始终运行
        // =========================================================

        robot.shooter1.setPower(
                shooterPower
        );


        robot.shooter2.setPower(
                shooterPower
        );


        // =========================================================
        // Shooter Ready
        // =========================================================

        boolean shooterReady =
                Math.abs(
                        error
                )
                        <= RPM_TOLERANCE;


        // =========================================================
        // Intake / Feed / Block
        // =========================================================


        // ---------------------------------------------------------
        // 左 Bumper：吐球
        // ---------------------------------------------------------

        if (
                gamepad1.left_bumper
        ) {

            robot.intake.setPower(
                    -1.0
            );


            robot.block.setPosition(
                    RobotHardware.BLOCK_CLOSE
            );


            feedUnlocked =
                    false;


            // ---------------------------------------------------------
            // 右 Trigger：发射
            // ---------------------------------------------------------

        } else if (
                gamepad1.right_trigger
                        > 0.1
        ) {


            /*
             * 第一球：
             *
             * 等待 RPM 达到目标 ± tolerance
             */

            if (
                    !feedUnlocked
                            && shooterReady
            ) {

                feedUnlocked =
                        true;
            }


            /*
             * 一旦第一次 Ready：
             *
             * 持续 Feed。
             *
             * 后续即使 RPM 因为射球下降，
             * 也不会停止。
             */

            if (
                    feedUnlocked
            ) {

                robot.block.setPosition(
                        RobotHardware.BLOCK_OPEN
                );


                // 使用前面统一定义的 FEED_POWER
                robot.intake.setPower(
                        FEED_POWER
                );

            } else {

                robot.block.setPosition(
                        RobotHardware.BLOCK_CLOSE
                );


                robot.intake.setPower(
                        0.0
                );
            }


            // ---------------------------------------------------------
            // 左 Trigger：吸球
            // ---------------------------------------------------------

        } else if (
                gamepad1.left_trigger
                        > 0.1
        ) {


            // 下一次发射重新等待 RPM

            feedUnlocked =
                    false;


            robot.block.setPosition(
                    RobotHardware.BLOCK_CLOSE
            );


            // 满球以后禁止继续吸球

            if (
                    ballsFull
            ) {

                robot.intake.setPower(
                        1.0
                );

            } else {

                // 正常吸球保持全速
                robot.intake.setPower(
                        1.0
                );
            }


            // ---------------------------------------------------------
            // 无操作
            // ---------------------------------------------------------

        } else {

            feedUnlocked =
                    false;


            robot.intake.setPower(
                    0.0
            );


            robot.block.setPosition(
                    RobotHardware.BLOCK_CLOSE
            );
        }


        // =========================================================
        // Driver Station Telemetry
        // =========================================================

        telemetry.addData(
                "驱动模式",
                fieldCentric
                        ? "场地中心"
                        : "机器人中心"
        );


        telemetry.addData(
                "朝向",
                "%.1f°",
                Math.toDegrees(
                        robot.getHeading()
                )
        );


        telemetry.addData(
                "",
                ""
        );


        // =====================================================
        // Shooter
        // =====================================================

        telemetry.addData(
                "Target RPM",
                "%.0f",
                TARGET_RPM
        );


        telemetry.addData(
                "Actual RPM",
                "%.0f",
                currentRPM
        );


        telemetry.addData(
                "RPM Error",
                "%.0f",
                error
        );


        telemetry.addData(
                "RPM Ready",
                shooterReady
                        ? "✓"
                        : "×"
        );


        telemetry.addData(
                "Feed Unlocked",
                feedUnlocked
                        ? "YES"
                        : "NO"
        );


        telemetry.addData(
                "Feed Power",
                "%.2f",
                FEED_POWER
        );


        telemetry.addData(
                "",
                ""
        );


        // =====================================================
        // Power / Voltage
        // =====================================================

        telemetry.addData(
                "Raw Power",
                "%.4f",
                rawShooterPower
        );


        telemetry.addData(
                "Shooter Power",
                "%.4f",
                shooterPower
        );


        telemetry.addData(
                "Battery Voltage",
                "%.2f V",
                batteryVoltage
        );


        telemetry.addData(
                "Nominal Voltage",
                "%.2f V",
                NOMINAL_VOLTAGE
        );


        telemetry.addData(
                "Voltage Comp",
                "%.4f",
                voltageCompensation
        );


        telemetry.addData(
                "",
                ""
        );


        // =====================================================
        // PID 参数
        // =====================================================

        telemetry.addData(
                "kF",
                "%.8f",
                kF
        );


        telemetry.addData(
                "kP",
                "%.8f",
                kP
        );


        telemetry.addData(
                "kI",
                "%.8f",
                kI
        );


        telemetry.addData(
                "kD",
                "%.8f",
                kD
        );


        telemetry.addData(
                "",
                ""
        );


        // =====================================================
        // PID Output
        // =====================================================

        telemetry.addData(
                "F Output",
                "%.4f",
                fOutput
        );


        telemetry.addData(
                "P Output",
                "%.4f",
                pOutput
        );


        telemetry.addData(
                "I Output",
                "%.4f",
                iOutput
        );


        telemetry.addData(
                "D Output",
                "%.4f",
                dOutput
        );


        telemetry.addData(
                "",
                ""
        );


        // =====================================================
        // 球状态
        // =====================================================

        telemetry.addData(
                "传感器 F",
                ballAtF
                        ? "✓ 有球"
                        : "无球"
        );


        telemetry.addData(
                "传感器 M",
                ballAtM
                        ? "✓ 有球"
                        : "无球"
        );


        telemetry.addData(
                "传感器 B",
                ballAtB
                        ? "✓ 有球"
                        : "无球"
        );


        telemetry.addData(
                "球满状态",
                ballsFull
                        ? "✓ 满"
                        : "未满"
        );


        telemetry.update();


        // =========================================================
        // FTC Dashboard
        // =========================================================

        TelemetryPacket packet =
                new TelemetryPacket();


        // =====================================================
        // RPM
        // =====================================================

        packet.put(
                "TargetRPM",
                TARGET_RPM
        );


        packet.put(
                "ActualRPM",
                currentRPM
        );


        packet.put(
                "Error",
                error
        );


        // =====================================================
        // Power
        // =====================================================

        packet.put(
                "RawPower",
                rawShooterPower
        );


        packet.put(
                "Power",
                shooterPower
        );


        // =====================================================
        // Voltage
        // =====================================================

        packet.put(
                "BatteryVoltage",
                batteryVoltage
        );


        packet.put(
                "NominalVoltage",
                NOMINAL_VOLTAGE
        );


        packet.put(
                "VoltageComp",
                voltageCompensation
        );


        // =====================================================
        // PID Outputs
        // =====================================================

        packet.put(
                "F_Output",
                fOutput
        );


        packet.put(
                "P_Output",
                pOutput
        );


        packet.put(
                "I_Output",
                iOutput
        );


        packet.put(
                "D_Output",
                dOutput
        );


        // =====================================================
        // Feed
        // =====================================================

        packet.put(
                "Feed",
                feedUnlocked
                        ? 1
                        : 0
        );


        packet.put(
                "FeedPower",
                FEED_POWER
        );


        // =====================================================
        // 当前 PID 参数
        // =====================================================

        packet.put(
                "Current_kF",
                kF
        );


        packet.put(
                "Current_kP",
                kP
        );


        packet.put(
                "Current_kI",
                kI
        );


        packet.put(
                "Current_kD",
                kD
        );


        // 发送 Dashboard

        FtcDashboard
                .getInstance()
                .sendTelemetryPacket(
                        packet
                );
    }


    // =========================================================
    // Reset PID
    // =========================================================

    private void resetPID() {

        integral =
                0.0;


        lastError =
                0.0;


        lastTime =
                System.nanoTime();
    }


    // =========================================================
    // Stop
    // =========================================================

    @Override
    public void stop() {

        robot.intake.setPower(
                0.0
        );


        robot.shooter1.setPower(
                0.0
        );


        robot.shooter2.setPower(
                0.0
        );


        robot.block.setPosition(
                RobotHardware.BLOCK_CLOSE
        );


        robot.ledL.setPosition(
                0.0
        );


        robot.ledR.setPosition(
                0.0
        );


        feedUnlocked =
                false;
    }
}