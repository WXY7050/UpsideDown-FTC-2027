package org.firstinspires.ftc.teamcode.upsidedown.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.upsidedown.hardware.RobotHardware;


@TeleOp(
        name = "Shooter PID 手柄调参",
        group = "Tuning"
)
public class ShooterPIDGamepadTuner extends OpMode {


    // =========================================================
    // Hardware
    // =========================================================

    private RobotHardware robot;

    private VoltageSensor voltageSensor;


    // =========================================================
    // 默认参数
    // =========================================================

    private static final double DEFAULT_TARGET_RPM =
            3725;

    private static final double DEFAULT_RPM_TOLERANCE =
            35;

    private static final double DEFAULT_KF =
            0.00017020;

    private static final double DEFAULT_KP =
            0.000446;

    private static final double DEFAULT_KI =
            0.0;

    private static final double DEFAULT_KD =
            0.000001;

    private static final double DEFAULT_FEED_POWER =
            1.0;


    // =========================================================
    // 可调参数
    // =========================================================

    public static double TARGET_RPM =
            DEFAULT_TARGET_RPM;

    public static double RPM_TOLERANCE =
            DEFAULT_RPM_TOLERANCE;

    public static double kF =
            DEFAULT_KF;

    public static double kP =
            DEFAULT_KP;

    public static double kI =
            DEFAULT_KI;

    public static double kD =
            DEFAULT_KD;


    // =========================================================
    // Feed Power
    // =========================================================

    /*
     * 只控制“发射时”的 Intake Feed 速度。
     *
     * 左 Trigger 正常吸球仍然使用 1.0。
     *
     * 推荐测试：
     *
     * 1.00
     * 0.95
     * 0.90
     * 0.85
     */

    public static double FEED_POWER =
            DEFAULT_FEED_POWER;


    // =========================================================
    // 电压补偿
    // =========================================================

    public static double NOMINAL_VOLTAGE =
            12.8;


    // =========================================================
    // Integral Anti-Windup
    // =========================================================

    public static double MAX_INTEGRAL_OUTPUT =
            0.20;


    // =========================================================
    // Target RPM 限制
    // =========================================================

    private static final double MIN_TARGET_RPM =
            0;

    private static final double MAX_TARGET_RPM =
            7000;


    // =========================================================
    // PID 内部变量
    // =========================================================

    private double integral =
            0;

    private double lastError =
            0;

    private long lastTime =
            0;


    // =========================================================
    // Shooter 状态
    // =========================================================

    private boolean shooterEnabled =
            false;

    private boolean feedUnlocked =
            false;


    // =========================================================
    // 可选择参数
    // =========================================================

    private enum TuneParameter {

        TARGET_RPM,

        RPM_TOLERANCE,

        KF,

        KP,

        KI,

        KD,

        FEED_POWER,

        NOMINAL_VOLTAGE
    }


    private TuneParameter selectedParameter =
            TuneParameter.FEED_POWER;


    // =========================================================
    // Step Multiplier
    // =========================================================

    /*
     * Y = ×10
     * X = ÷10
     */

    private double stepMultiplier =
            1.0;


    // =========================================================
    // Button Edge Detection
    // =========================================================

    private boolean lastDpadLeft =
            false;

    private boolean lastDpadRight =
            false;

    private boolean lastDpadUp =
            false;

    private boolean lastDpadDown =
            false;

    private boolean lastX =
            false;

    private boolean lastY =
            false;

    private boolean lastA =
            false;

    private boolean lastB =
            false;

    private boolean lastRightBumper =
            false;


    // =========================================================
    // INIT
    // =========================================================

    @Override
    public void init() {

        robot =
                new RobotHardware();

        robot.init(
                hardwareMap
        );


        // =====================================================
        // 电压传感器
        // =====================================================

        voltageSensor =
                hardwareMap.voltageSensor
                        .iterator()
                        .next();


        // =====================================================
        // Motors / Servo 初始状态
        // =====================================================

        robot.shooter1.setPower(
                0
        );

        robot.shooter2.setPower(
                0
        );

        robot.intake.setPower(
                0
        );

        robot.block.setPosition(
                RobotHardware.BLOCK_CLOSE
        );


        telemetry.addLine(
                "Shooter PID + Feed Tuner"
        );

        telemetry.addLine(
                ""
        );

        telemetry.addLine(
                "A = Shooter ON / OFF"
        );

        telemetry.addLine(
                "B = 恢复默认参数"
        );

        telemetry.addLine(
                ""
        );

        telemetry.addLine(
                "← → = 选择参数"
        );

        telemetry.addLine(
                "↑ ↓ = 修改参数"
        );

        telemetry.addLine(
                "Y = Step ×10"
        );

        telemetry.addLine(
                "X = Step ÷10"
        );

        telemetry.addLine(
                ""
        );

        telemetry.addLine(
                "RB = Reset PID"
        );

        telemetry.addLine(
                ""
        );

        telemetry.addLine(
                "RT = 发射"
        );

        telemetry.addLine(
                "LT = Intake"
        );

        telemetry.addLine(
                "LB = Reverse"
        );

        telemetry.update();
    }


    // =========================================================
    // START
    // =========================================================

    @Override
    public void start() {

        resetPID();

        shooterEnabled =
                false;

        feedUnlocked =
                false;
    }


    // =========================================================
    // LOOP
    // =========================================================

    @Override
    public void loop() {


        // =====================================================
        // Shooter ON / OFF
        // =====================================================

        if (
                gamepad1.a
                        && !lastA
        ) {

            shooterEnabled =
                    !shooterEnabled;

            resetPID();

            feedUnlocked =
                    false;
        }

        lastA =
                gamepad1.a;


        // =====================================================
        // 恢复默认参数
        // =====================================================

        if (
                gamepad1.b
                        && !lastB
        ) {

            resetAllParameters();
        }

        lastB =
                gamepad1.b;


        // =====================================================
        // 参数选择
        // =====================================================

        if (
                gamepad1.dpad_right
                        && !lastDpadRight
        ) {

            selectNextParameter();
        }


        if (
                gamepad1.dpad_left
                        && !lastDpadLeft
        ) {

            selectPreviousParameter();
        }


        lastDpadRight =
                gamepad1.dpad_right;

        lastDpadLeft =
                gamepad1.dpad_left;


        // =====================================================
        // Step Multiplier
        // =====================================================

        if (
                gamepad1.y
                        && !lastY
        ) {

            stepMultiplier *=
                    10.0;

            if (
                    stepMultiplier > 1000
            ) {

                stepMultiplier =
                        1000;
            }
        }


        if (
                gamepad1.x
                        && !lastX
        ) {

            stepMultiplier /=
                    10.0;

            if (
                    stepMultiplier < 0.001
            ) {

                stepMultiplier =
                        0.001;
            }
        }


        lastY =
                gamepad1.y;

        lastX =
                gamepad1.x;


        // =====================================================
        // 修改当前参数
        // =====================================================

        if (
                gamepad1.dpad_up
                        && !lastDpadUp
        ) {

            changeSelectedParameter(
                    +1
            );
        }


        if (
                gamepad1.dpad_down
                        && !lastDpadDown
        ) {

            changeSelectedParameter(
                    -1
            );
        }


        lastDpadUp =
                gamepad1.dpad_up;

        lastDpadDown =
                gamepad1.dpad_down;


        // =====================================================
        // Reset PID
        // =====================================================

        if (
                gamepad1.right_bumper
                        && !lastRightBumper
        ) {

            resetPID();

            feedUnlocked =
                    false;
        }


        lastRightBumper =
                gamepad1.right_bumper;


        // =====================================================
        // 当前电池电压
        // =====================================================

        double batteryVoltage =
                voltageSensor.getVoltage();


        if (
                batteryVoltage < 1.0
        ) {

            batteryVoltage =
                    NOMINAL_VOLTAGE;
        }


        // =====================================================
        // 当前 RPM
        // =====================================================

        double currentRPM =
                robot.getShooterRPM();


        double error =
                TARGET_RPM
                        - currentRPM;


        // =====================================================
        // PID Timing
        // =====================================================

        long currentTime =
                System.nanoTime();


        double dt =
                (
                        currentTime
                                - lastTime
                )
                        / 1_000_000_000.0;


        lastTime =
                currentTime;


        if (
                dt <= 0
                        || dt > 0.2
        ) {

            dt =
                    0.02;
        }


        // =====================================================
        // Integral
        // =====================================================

        integral +=
                error * dt;


        if (
                kI != 0
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
                    0;
        }


        // =====================================================
        // Derivative
        // =====================================================

        double derivative =
                (
                        error
                                - lastError
                )
                        / dt;


        lastError =
                error;


        // =====================================================
        // PIDF Output
        // =====================================================

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


        // =====================================================
        // Raw Power
        // =====================================================

        double rawPower =
                fOutput
                        + pOutput
                        + iOutput
                        + dOutput;


        // =====================================================
        // Voltage Compensation
        // =====================================================

        double voltageCompensation =
                NOMINAL_VOLTAGE
                        / batteryVoltage;


        // =====================================================
        // Final Shooter Power
        // =====================================================

        double shooterPower =
                rawPower
                        * voltageCompensation;


        shooterPower =
                Math.max(
                        0,
                        Math.min(
                                1,
                                shooterPower
                        )
                );


        // =====================================================
        // Shooter Output
        // =====================================================

        if (
                shooterEnabled
        ) {

            robot.shooter1.setPower(
                    shooterPower
            );

            robot.shooter2.setPower(
                    shooterPower
            );

        } else {

            robot.shooter1.setPower(
                    0
            );

            robot.shooter2.setPower(
                    0
            );

            feedUnlocked =
                    false;
        }


        // =====================================================
        // Shooter Ready
        // =====================================================

        boolean shooterReady =
                shooterEnabled
                        && Math.abs(
                        error
                )
                        <= RPM_TOLERANCE;


        // =====================================================
        // Intake / Shoot
        // =====================================================


        // -----------------------------------------------------
        // LB = Reverse
        // -----------------------------------------------------

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


            // -----------------------------------------------------
            // RT = Shoot
            // -----------------------------------------------------

        } else if (
                gamepad1.right_trigger > 0.1
        ) {


            /*
             * 第一颗球：
             *
             * 必须等待 RPM 达到目标 ± tolerance。
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
             * 这里使用可调的 FEED_POWER。
             */

            if (
                    feedUnlocked
            ) {

                robot.block.setPosition(
                        RobotHardware.BLOCK_OPEN
                );


                robot.intake.setPower(
                        FEED_POWER
                );

            } else {

                robot.block.setPosition(
                        RobotHardware.BLOCK_CLOSE
                );


                robot.intake.setPower(
                        0
                );
            }


            // -----------------------------------------------------
            // LT = Intake
            // -----------------------------------------------------

        } else if (
                gamepad1.left_trigger > 0.1
        ) {

            feedUnlocked =
                    false;


            robot.block.setPosition(
                    RobotHardware.BLOCK_CLOSE
            );


            /*
             * 正常 Intake 不受 FEED_POWER 影响。
             */

            robot.intake.setPower(
                    1.0
            );


            // -----------------------------------------------------
            // Nothing
            // -----------------------------------------------------

        } else {

            feedUnlocked =
                    false;


            robot.block.setPosition(
                    RobotHardware.BLOCK_CLOSE
            );


            robot.intake.setPower(
                    0
            );
        }


        // =====================================================
        // Driver Station Telemetry
        // =====================================================

        telemetry.addLine(
                "===== SHOOTER PID + FEED TUNER ====="
        );


        telemetry.addData(
                "Shooter",
                shooterEnabled
                        ? "ON"
                        : "OFF"
        );


        telemetry.addData(
                "RPM Ready",
                shooterReady
                        ? "YES"
                        : "NO"
        );


        telemetry.addData(
                "Feed",
                feedUnlocked
                        ? "ON"
                        : "OFF"
        );


        telemetry.addLine(
                ""
        );


        // =====================================================
        // 当前调参
        // =====================================================

        telemetry.addLine(
                "------ 当前调参 ------"
        );


        telemetry.addData(
                "Selected",
                getSelectedParameterName()
        );


        telemetry.addData(
                "Value",
                getSelectedParameterValue()
        );


        telemetry.addData(
                "Step",
                formatStep(
                        getCurrentStep()
                )
        );


        telemetry.addData(
                "Step Multiplier",
                stepMultiplier
        );


        telemetry.addLine(
                ""
        );


        // =====================================================
        // PID 参数
        // =====================================================

        telemetry.addLine(
                "------ Parameters ------"
        );


        telemetry.addData(
                "TARGET RPM",
                "%.0f",
                TARGET_RPM
        );


        telemetry.addData(
                "Tolerance",
                "%.0f",
                RPM_TOLERANCE
        );


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
                "Feed Power",
                "%.2f",
                FEED_POWER
        );


        telemetry.addLine(
                ""
        );


        // =====================================================
        // 电压
        // =====================================================

        telemetry.addLine(
                "------ Voltage ------"
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


        telemetry.addLine(
                ""
        );


        // =====================================================
        // Shooter
        // =====================================================

        telemetry.addLine(
                "------ Shooter ------"
        );


        telemetry.addData(
                "Target RPM",
                "%.1f",
                TARGET_RPM
        );


        telemetry.addData(
                "Actual RPM",
                "%.1f",
                currentRPM
        );


        telemetry.addData(
                "Error",
                "%.1f",
                error
        );


        telemetry.addData(
                "Raw Power",
                "%.4f",
                rawPower
        );


        telemetry.addData(
                "Final Power",
                "%.4f",
                shooterPower
        );


        telemetry.addLine(
                ""
        );


        // =====================================================
        // PID Output
        // =====================================================

        telemetry.addLine(
                "------ PID Output ------"
        );


        telemetry.addData(
                "F",
                "%.4f",
                fOutput
        );


        telemetry.addData(
                "P",
                "%.4f",
                pOutput
        );


        telemetry.addData(
                "I",
                "%.4f",
                iOutput
        );


        telemetry.addData(
                "D",
                "%.4f",
                dOutput
        );


        telemetry.addLine(
                ""
        );


        telemetry.addLine(
                "← → Select"
        );

        telemetry.addLine(
                "↑ ↓ Change"
        );

        telemetry.addLine(
                "Y ×10 | X ÷10"
        );

        telemetry.addLine(
                "A Shooter ON/OFF"
        );

        telemetry.addLine(
                "B Reset Defaults"
        );

        telemetry.addLine(
                "RB Reset PID"
        );


        telemetry.update();


        // =====================================================
        // FTC Dashboard
        // =====================================================

        TelemetryPacket packet =
                new TelemetryPacket();


        // RPM

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


        // Voltage

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


        // Power

        packet.put(
                "RawPower",
                rawPower
        );


        packet.put(
                "Power",
                shooterEnabled
                        ? shooterPower
                        : 0
        );


        // Feed

        packet.put(
                "FeedPower",
                FEED_POWER
        );


        packet.put(
                "Feed",
                feedUnlocked
                        ? 1
                        : 0
        );


        // PID Outputs

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


        FtcDashboard
                .getInstance()
                .sendTelemetryPacket(
                        packet
                );
    }


    // =========================================================
    // Next Parameter
    // =========================================================

    private void selectNextParameter() {

        TuneParameter[] parameters =
                TuneParameter.values();


        int next =
                selectedParameter.ordinal()
                        + 1;


        if (
                next >= parameters.length
        ) {

            next =
                    0;
        }


        selectedParameter =
                parameters[next];
    }


    // =========================================================
    // Previous Parameter
    // =========================================================

    private void selectPreviousParameter() {

        TuneParameter[] parameters =
                TuneParameter.values();


        int previous =
                selectedParameter.ordinal()
                        - 1;


        if (
                previous < 0
        ) {

            previous =
                    parameters.length
                            - 1;
        }


        selectedParameter =
                parameters[previous];
    }


    // =========================================================
    // 修改参数
    // =========================================================

    private void changeSelectedParameter(
            int direction
    ) {

        double step =
                getCurrentStep();


        switch (
                selectedParameter
        ) {


            case TARGET_RPM:

                TARGET_RPM +=
                        direction
                                * step;


                TARGET_RPM =
                        Math.max(
                                MIN_TARGET_RPM,
                                Math.min(
                                        MAX_TARGET_RPM,
                                        TARGET_RPM
                                )
                        );


                resetPID();

                feedUnlocked =
                        false;

                break;


            case RPM_TOLERANCE:

                RPM_TOLERANCE +=
                        direction
                                * step;


                RPM_TOLERANCE =
                        Math.max(
                                0,
                                RPM_TOLERANCE
                        );

                break;


            case KF:

                kF +=
                        direction
                                * step;


                kF =
                        Math.max(
                                0,
                                kF
                        );

                break;


            case KP:

                kP +=
                        direction
                                * step;


                kP =
                        Math.max(
                                0,
                                kP
                        );

                break;


            case KI:

                kI +=
                        direction
                                * step;


                kI =
                        Math.max(
                                0,
                                kI
                        );


                resetPID();

                break;


            case KD:

                kD +=
                        direction
                                * step;


                kD =
                        Math.max(
                                0,
                                kD
                        );

                break;


            case FEED_POWER:

                FEED_POWER +=
                        direction
                                * step;


                FEED_POWER =
                        Math.max(
                                0.0,
                                Math.min(
                                        1.0,
                                        FEED_POWER
                                )
                        );

                break;


            case NOMINAL_VOLTAGE:

                NOMINAL_VOLTAGE +=
                        direction
                                * step;


                NOMINAL_VOLTAGE =
                        Math.max(
                                8.0,
                                Math.min(
                                        14.5,
                                        NOMINAL_VOLTAGE
                                )
                        );

                break;
        }
    }


    // =========================================================
    // 每个参数基础步长
    // =========================================================

    private double getCurrentStep() {

        double baseStep;


        switch (
                selectedParameter
        ) {


            case TARGET_RPM:

                baseStep =
                        25;

                break;


            case RPM_TOLERANCE:

                baseStep =
                        5;

                break;


            case KF:

                baseStep =
                        0.000001;

                break;


            case KP:

                baseStep =
                        0.00001;

                break;


            case KI:

                baseStep =
                        0.000001;

                break;


            case KD:

                baseStep =
                        0.000001;

                break;


            case FEED_POWER:

                baseStep =
                        0.05;

                break;


            case NOMINAL_VOLTAGE:

                baseStep =
                        0.1;

                break;


            default:

                baseStep =
                        1;

                break;
        }


        return baseStep
                * stepMultiplier;
    }


    // =========================================================
    // 参数名称
    // =========================================================

    private String getSelectedParameterName() {

        switch (
                selectedParameter
        ) {


            case TARGET_RPM:

                return "TARGET_RPM";


            case RPM_TOLERANCE:

                return "RPM_TOLERANCE";


            case KF:

                return "kF";


            case KP:

                return "kP";


            case KI:

                return "kI";


            case KD:

                return "kD";


            case FEED_POWER:

                return "FEED_POWER";


            case NOMINAL_VOLTAGE:

                return "NOMINAL_VOLTAGE";


            default:

                return "";
        }
    }


    // =========================================================
    // 参数当前值
    // =========================================================

    private String getSelectedParameterValue() {

        switch (
                selectedParameter
        ) {


            case TARGET_RPM:

                return String.format(
                        "%.0f RPM",
                        TARGET_RPM
                );


            case RPM_TOLERANCE:

                return String.format(
                        "%.0f RPM",
                        RPM_TOLERANCE
                );


            case KF:

                return String.format(
                        "%.8f",
                        kF
                );


            case KP:

                return String.format(
                        "%.8f",
                        kP
                );


            case KI:

                return String.format(
                        "%.8f",
                        kI
                );


            case KD:

                return String.format(
                        "%.8f",
                        kD
                );


            case FEED_POWER:

                return String.format(
                        "%.2f",
                        FEED_POWER
                );


            case NOMINAL_VOLTAGE:

                return String.format(
                        "%.2f V",
                        NOMINAL_VOLTAGE
                );


            default:

                return "";
        }
    }


    // =========================================================
    // Step 显示格式
    // =========================================================

    private String formatStep(
            double step
    ) {

        if (
                selectedParameter
                        == TuneParameter.TARGET_RPM
                        ||
                        selectedParameter
                                == TuneParameter.RPM_TOLERANCE
        ) {

            return String.format(
                    "%.1f RPM",
                    step
            );

        } else if (
                selectedParameter
                        == TuneParameter.NOMINAL_VOLTAGE
        ) {

            return String.format(
                    "%.2f V",
                    step
            );

        } else if (
                selectedParameter
                        == TuneParameter.FEED_POWER
        ) {

            return String.format(
                    "%.2f",
                    step
            );

        } else {

            return String.format(
                    "%.8f",
                    step
            );
        }
    }


    // =========================================================
    // Reset 默认值
    // =========================================================

    private void resetAllParameters() {

        TARGET_RPM =
                DEFAULT_TARGET_RPM;


        RPM_TOLERANCE =
                DEFAULT_RPM_TOLERANCE;


        kF =
                DEFAULT_KF;


        kP =
                DEFAULT_KP;


        kI =
                DEFAULT_KI;


        kD =
                DEFAULT_KD;


        FEED_POWER =
                DEFAULT_FEED_POWER;


        NOMINAL_VOLTAGE =
                12.8;


        stepMultiplier =
                1.0;


        feedUnlocked =
                false;


        resetPID();
    }


    // =========================================================
    // Reset PID
    // =========================================================

    private void resetPID() {

        integral =
                0;


        lastError =
                0;


        lastTime =
                System.nanoTime();
    }


    // =========================================================
    // STOP
    // =========================================================

    @Override
    public void stop() {

        robot.shooter1.setPower(
                0
        );


        robot.shooter2.setPower(
                0
        );


        robot.intake.setPower(
                0
        );


        robot.block.setPosition(
                RobotHardware.BLOCK_CLOSE
        );


        shooterEnabled =
                false;


        feedUnlocked =
                false;
    }
}