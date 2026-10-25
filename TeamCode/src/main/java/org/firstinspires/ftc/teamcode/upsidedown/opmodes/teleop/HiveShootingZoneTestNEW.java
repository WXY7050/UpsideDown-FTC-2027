package org.firstinspires.ftc.teamcode.upsidedown.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.upsidedown.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.upsidedown.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.upsidedown.subsystems.FlowerSubsystem;
import org.firstinspires.ftc.teamcode.upsidedown.subsystems.LEDSubsystem;

@Disabled
@TeleOp(name = "新测试Hive Shooting Zone Test", group = "Test")
public class HiveShootingZoneTestNEW extends OpMode {


    // ============================================================
    // Pedro / Robot
    // ============================================================

    private Follower follower;
    private RobotHardware robot;
    private VoltageSensor voltageSensor;


    // ============================================================
    // 场地起始 Pose
    //
    // 场地左下角 = (0,0)
    //
    // X 向右
    // Y 向上
    //
    // 测试启动时：
    // 机器人左侧贴左墙
    // 机器人后侧贴下墙
    // 车头朝上 = 90°
    //
    // 机器人左右宽 = 12.441"
    // 机器人前后长 = 12.835"
    // ============================================================

    private static final double START_X = 12.441 / 2.0;   // 6.2205
    private static final double START_Y = 12.835 / 2.0;   // 6.4175
    private static final double START_HEADING_DEG = 90.0;


    // ============================================================
    // 左下 RED HIVE
    // ============================================================

    private static final double HIVE_X = 57.0;
    private static final double HIVE_Y = 55.0;

    // 左下 HIVE 开口朝下
    private static final double HIVE_OPEN_HEADING = 270.0;


    // ============================================================
    // 第一版实测模型（先验证核心区域）
    //
    // 这组锚点来自 49 in 左右的实测成功点。
    // 先只在 Sector -30° ~ +40°、Distance 45 ~ 55 in 内验证，
    // 验证稳定后再扩展到完整 Shooting Zone。
    // ============================================================

    private static final double MODEL_MIN_SECTOR = -30.0;
    private static final double MODEL_MAX_SECTOR = 40.0;
    private static final double MODEL_MIN_DISTANCE = 45.0;
    private static final double MODEL_MAX_DISTANCE = 55.0;

    private static final double MODEL_BASE_DISTANCE = 49.0;
    private static final double MODEL_RPM_DISTANCE_SLOPE = 6.0; // RPM / inch

    // Sector Error 锚点（度）
    private static final double[] MODEL_SECTOR = {
            -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 40.0
    };

    // 相对于“几何瞄准 HIVE 中心”的 Heading 修正（度）
    private static final double[] MODEL_AIM_OFFSET = {
            21.4, 12.8, 6.4, 0.7, 8.0, 0.4, -0.9, -1.9
    };

    // 49 in 左右对应的实测 RPM 锚点
    private static final double[] MODEL_BASE_RPM = {
            3800, 3775, 3725, 3725, 3725, 3675, 3675, 3675
    };


    // ============================================================
    // Shooter 参数
    // ============================================================

    public static double TARGET_RPM = 3775;

    public static double RPM_TOLERANCE = 35;

    public static final double FEED_POWER = 0.45;

    // 每按一下 D-pad 改多少 RPM
    public static final double RPM_STEP = 25.0;


    // ============================================================
    // Shooter PIDF
    // ============================================================

    public static double kF = 0.00017020;

    public static double kP = 0.000446;

    public static double kI = 0.0;

    public static double kD = 0.000001;

    public static double MAX_INTEGRAL_OUTPUT = 0.20;


    // ============================================================
    // Voltage Compensation
    // ============================================================

    public static final double NOMINAL_VOLTAGE = 12.9;


    // ============================================================
    // PID 内部状态
    // ============================================================

    private double integral = 0.0;

    private double lastError = 0.0;

    private long lastTime = 0;


    // ============================================================
    // Feed 状态
    // ============================================================

    private boolean feedUnlocked = false;


    // ============================================================
    // D-pad 边沿检测
    // ============================================================

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastA = false;


    // ============================================================
    // Init
    // ============================================================


    @Override
    public void init() {

        // ============================================================
        // 先初始化 RobotHardware
        // 如果 RobotHardware 会配置 Pinpoint，
        // 让它先做完
        // ============================================================

        robot = new RobotHardware();
        robot.init(hardwareMap);


        // ============================================================
        // 最后再创建 Pedro Follower
        //
        // 这样 Pedro 的 PinpointConfig 是最后生效的一套配置
        // ============================================================

        follower = Constants.create(hardwareMap);


        // ============================================================
        // Voltage
        // ============================================================

        voltageSensor =
                hardwareMap.voltageSensor
                        .iterator()
                        .next();


        // ============================================================
        // 默认状态
        // ============================================================

        robot.intake.setPower(0.0);

        robot.block.setPosition(
                org.firstinspires.ftc.teamcode.upsidedown.RobotConstants.BLOCK_CLOSE
        );
    }


    // ============================================================
    // Start
    // ============================================================


    @Override
    public void start() {

        resetPID();

        feedUnlocked = false;

        Pose startPose =
                new Pose(
                        START_X,
                        START_Y,
                        Math.toRadians(START_HEADING_DEG)
                );

        follower.setPose(startPose);
    }


    // ============================================================
    // Loop
    // ============================================================

    @Override
    public void loop() {


        // ========================================================
        // Pedro Localization
        // ========================================================

        follower.update();

        Pose robotPose =
                follower.pose();


        double robotX =
                robotPose.x();

        double robotY =
                robotPose.y();

        double robotHeadingDeg =
                normalize360(
                        Math.toDegrees(
                                robotPose.heading()
                        )
                );


        // ========================================================
        // RPM 调整
        // ========================================================

        if (
                gamepad1.dpad_up
                        && !lastDpadUp
        ) {

            TARGET_RPM += RPM_STEP;
        }


        if (
                gamepad1.dpad_down
                        && !lastDpadDown
        ) {

            TARGET_RPM -= RPM_STEP;
        }


        // 避免输入离谱数值

        TARGET_RPM =
                Math.max(
                        0,
                        Math.min(
                                6000,
                                TARGET_RPM
                        )
                );


        lastDpadUp =
                gamepad1.dpad_up;

        lastDpadDown =
                gamepad1.dpad_down;


        // ========================================================
        // HIVE -> Robot 几何
        // ========================================================

        double hiveToRobotX =
                robotX - HIVE_X;

        double hiveToRobotY =
                robotY - HIVE_Y;


        // 机器人距离 HIVE 的距离

        double distanceToHive =
                Math.hypot(
                        hiveToRobotX,
                        hiveToRobotY
                );


        // 机器人位于 HIVE 的哪个方向

        double angleFromHive =
                Math.toDegrees(
                        Math.atan2(
                                hiveToRobotY,
                                hiveToRobotX
                        )
                );

        angleFromHive =
                normalize360(
                        angleFromHive
                );


        // ========================================================
        // Robot -> HIVE
        //
        // 机器人真正应该朝向的 Heading
        // ========================================================

        double targetHeading =
                Math.toDegrees(
                        Math.atan2(
                                HIVE_Y - robotY,
                                HIVE_X - robotX
                        )
                );

        targetHeading =
                normalize360(
                        targetHeading
                );


        // ========================================================
        // 当前 Heading Error
        // ========================================================

        double headingError =
                normalize180(
                        targetHeading
                                - robotHeadingDeg
                );


        // ========================================================
        // HIVE 开口扇区
        // ========================================================

        double sectorError =
                normalize180(
                        angleFromHive
                                - HIVE_OPEN_HEADING
                );


        // ========================================================
        // 第一版推荐模型
        // ========================================================

        boolean modelValid =
                sectorError >= MODEL_MIN_SECTOR
                        && sectorError <= MODEL_MAX_SECTOR
                        && distanceToHive >= MODEL_MIN_DISTANCE
                        && distanceToHive <= MODEL_MAX_DISTANCE;

        // 根据 Sector Error 在线性锚点之间插值
        double recommendedAimOffset =
                interpolateBySector(
                        sectorError,
                        MODEL_AIM_OFFSET
                );

        double recommendedHeading =
                normalize360(
                        targetHeading + recommendedAimOffset
                );

        double recommendedHeadingError =
                normalize180(
                        recommendedHeading - robotHeadingDeg
                );

        double recommendedBaseRPM =
                interpolateBySector(
                        sectorError,
                        MODEL_BASE_RPM
                );

        double recommendedRPM =
                recommendedBaseRPM
                        + MODEL_RPM_DISTANCE_SLOPE
                        * (distanceToHive - MODEL_BASE_DISTANCE);

        // 与当前测试习惯一致，推荐值取最近的 25 RPM
        recommendedRPM =
                Math.round(recommendedRPM / 25.0) * 25.0;

        // A：把当前模型推荐 RPM 载入 TARGET_RPM
        if (
                gamepad1.a
                        && !lastA
                        && modelValid
        ) {
            TARGET_RPM = recommendedRPM;
            resetPID();
        }

        lastA = gamepad1.a;


        // ========================================================
        // Shooter RPM
        // ========================================================

        double currentRPM =
                robot.getShooterRPM();


        double error =
                TARGET_RPM
                        - currentRPM;


        // ========================================================
        // PID Time
        // ========================================================

        long currentTime =
                System.nanoTime();


        double dt =
                (currentTime - lastTime)
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


        // ========================================================
        // Integral
        // ========================================================

        integral +=
                error * dt;


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


        // ========================================================
        // Derivative
        // ========================================================

        double derivative =
                (error - lastError)
                        / dt;


        lastError =
                error;


        // ========================================================
        // PIDF Output
        // ========================================================

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


        double rawShooterPower =
                fOutput
                        + pOutput
                        + iOutput
                        + dOutput;


        // ========================================================
        // Battery Voltage
        // ========================================================

        double batteryVoltage =
                voltageSensor.getVoltage();


        if (
                batteryVoltage < 1.0
        ) {

            batteryVoltage =
                    NOMINAL_VOLTAGE;
        }


        // ========================================================
        // Voltage Compensation
        // ========================================================

        double voltageCompensation =
                NOMINAL_VOLTAGE
                        / batteryVoltage;


        double shooterPower =
                rawShooterPower
                        * voltageCompensation;


        shooterPower =
                Math.max(
                        0.0,
                        Math.min(
                                1.0,
                                shooterPower
                        )
                );


        // ========================================================
        // Shooter 始终运行
        // ========================================================

        robot.shooter1.setPower(
                shooterPower
        );

        robot.shooter2.setPower(
                shooterPower
        );


        // ========================================================
        // Shooter Ready
        // ========================================================

        boolean shooterReady =
                Math.abs(
                        error
                )
                        <= RPM_TOLERANCE;


        // ========================================================
        // Intake / Feed
        // ========================================================


        // --------------------------------------------------------
        // 左 Bumper：吐球
        // --------------------------------------------------------

        if (
                gamepad1.left_bumper
        ) {

            robot.intake.setPower(
                    -1.0
            );

            robot.block.setPosition(
                    org.firstinspires.ftc.teamcode.upsidedown.RobotConstants.BLOCK_CLOSE
            );

            feedUnlocked =
                    false;


            // --------------------------------------------------------
            // 右 Trigger：发射
            // --------------------------------------------------------

        } else if (
                gamepad1.right_trigger
                        > 0.1
        ) {


            // 第一球等待达到 RPM

            if (
                    !feedUnlocked
                            && shooterReady
            ) {

                feedUnlocked =
                        true;
            }


            // Ready 后持续 Feed

            if (
                    feedUnlocked
            ) {

                robot.block.setPosition(
                        org.firstinspires.ftc.teamcode.upsidedown.RobotConstants.BLOCK_OPEN
                );

                robot.intake.setPower(
                        FEED_POWER
                );

            } else {

                robot.block.setPosition(
                        org.firstinspires.ftc.teamcode.upsidedown.RobotConstants.BLOCK_CLOSE
                );

                robot.intake.setPower(
                        0.0
                );
            }


            // --------------------------------------------------------
            // 左 Trigger：吸球
            // --------------------------------------------------------

        } else if (
                gamepad1.left_trigger
                        > 0.1
        ) {

            feedUnlocked =
                    false;

            robot.block.setPosition(
                    org.firstinspires.ftc.teamcode.upsidedown.RobotConstants.BLOCK_CLOSE
            );

            robot.intake.setPower(
                    1.0
            );


            // --------------------------------------------------------
            // 无操作
            // --------------------------------------------------------

        } else {

            feedUnlocked =
                    false;

            robot.intake.setPower(
                    0.0
            );

            robot.block.setPosition(
                    org.firstinspires.ftc.teamcode.upsidedown.RobotConstants.BLOCK_CLOSE
            );
        }


        // ========================================================
        // Driver Station Telemetry
        // ========================================================

        telemetry.addLine(
                "========== HIVE TEST =========="
        );


        telemetry.addData(
                "Robot X",
                "%.2f",
                robotX
        );

        telemetry.addData(
                "Robot Y",
                "%.2f",
                robotY
        );

        telemetry.addData(
                "Robot Heading",
                "%.1f°",
                robotHeadingDeg
        );


        telemetry.addLine("");


        telemetry.addData(
                "HIVE",
                "(%.1f, %.1f)",
                HIVE_X,
                HIVE_Y
        );

        telemetry.addData(
                "Distance",
                "%.2f in",
                distanceToHive
        );

        telemetry.addData(
                "Angle From HIVE",
                "%.1f°",
                angleFromHive
        );

        telemetry.addData(
                "Target Heading",
                "%.1f°",
                targetHeading
        );

        telemetry.addData(
                "Heading Error",
                "%+.1f°",
                headingError
        );


        telemetry.addLine("");


        telemetry.addData(
                "Sector Center",
                "%.1f°",
                HIVE_OPEN_HEADING
        );

        telemetry.addData(
                "Sector Error",
                "%+.1f°",
                sectorError
        );

        telemetry.addData(
                "Model Valid",
                modelValid
                        ? "YES"
                        : "NO"
        );

        telemetry.addData(
                "Aim Offset",
                "%+.1f°",
                recommendedAimOffset
        );

        telemetry.addData(
                "Recommended Heading",
                "%.1f°",
                recommendedHeading
        );

        telemetry.addData(
                "Model Heading Error",
                "%+.1f°",
                recommendedHeadingError
        );

        telemetry.addData(
                "Recommended RPM",
                "%.0f",
                recommendedRPM
        );


        telemetry.addLine("");


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
                "Shooter Ready",
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


        telemetry.addLine("");


        telemetry.addData(
                "Battery",
                "%.2f V",
                batteryVoltage
        );

        telemetry.addData(
                "Shooter Power",
                "%.4f",
                shooterPower
        );


        telemetry.addLine("");

        telemetry.addLine(
                "Dpad ↑ ↓ : RPM +/-25"
        );

        telemetry.addLine(
                "RT : Fire"
        );

        telemetry.addLine(
                "LT : Intake"
        );

        telemetry.addLine(
                "A : Apply Recommended RPM"
        );

        telemetry.update();


        // ========================================================
        // Dashboard
        // ========================================================

        TelemetryPacket packet =
                new TelemetryPacket();


        packet.put(
                "RobotX",
                robotX
        );

        packet.put(
                "RobotY",
                robotY
        );

        packet.put(
                "RobotHeading",
                robotHeadingDeg
        );

        packet.put(
                "DistanceToHive",
                distanceToHive
        );

        packet.put(
                "AngleFromHive",
                angleFromHive
        );

        packet.put(
                "TargetHeading",
                targetHeading
        );

        packet.put(
                "HeadingError",
                headingError
        );

        packet.put(
                "SectorError",
                sectorError
        );

        packet.put(
                "ModelValid",
                modelValid ? 1 : 0
        );

        packet.put(
                "AimOffset",
                recommendedAimOffset
        );

        packet.put(
                "RecommendedHeading",
                recommendedHeading
        );

        packet.put(
                "RecommendedHeadingError",
                recommendedHeadingError
        );

        packet.put(
                "RecommendedRPM",
                recommendedRPM
        );

        packet.put(
                "TargetRPM",
                TARGET_RPM
        );

        packet.put(
                "ActualRPM",
                currentRPM
        );

        packet.put(
                "ShooterPower",
                shooterPower
        );

        packet.put(
                "BatteryVoltage",
                batteryVoltage
        );


        FtcDashboard
                .getInstance()
                .sendTelemetryPacket(
                        packet
                );
    }


    // ============================================================
    // Sector 线性插值
    // ============================================================

    private double interpolateBySector(
            double sectorError,
            double[] values
    ) {

        // 超出锚点时先夹到边界。
        // 当前 modelValid 会限制正式测试范围，
        // 这里 clamp 只是避免计算异常。
        if (sectorError <= MODEL_SECTOR[0]) {
            return values[0];
        }

        int last = MODEL_SECTOR.length - 1;

        if (sectorError >= MODEL_SECTOR[last]) {
            return values[last];
        }

        for (int i = 0; i < last; i++) {

            double s0 = MODEL_SECTOR[i];
            double s1 = MODEL_SECTOR[i + 1];

            if (sectorError >= s0 && sectorError <= s1) {

                double t =
                        (sectorError - s0)
                                / (s1 - s0);

                return values[i]
                        + t * (values[i + 1] - values[i]);
            }
        }

        return values[last];
    }


    // ============================================================
    // Reset PID
    // ============================================================

    private void resetPID() {

        integral =
                0.0;

        lastError =
                0.0;

        lastTime =
                System.nanoTime();
    }


    // ============================================================
    // Normalize 0 ~ 360
    // ============================================================

    private double normalize360(
            double angle
    ) {

        angle %=
                360.0;

        if (
                angle < 0
        ) {

            angle +=
                    360.0;
        }

        return angle;
    }


    // ============================================================
    // Normalize -180 ~ +180
    // ============================================================

    private double normalize180(
            double angle
    ) {

        angle %=
                360.0;

        if (
                angle > 180.0
        ) {

            angle -=
                    360.0;
        }

        if (
                angle < -180.0
        ) {

            angle +=
                    360.0;
        }

        return angle;
    }


    // ============================================================
    // Stop
    // ============================================================

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
                org.firstinspires.ftc.teamcode.upsidedown.RobotConstants.BLOCK_CLOSE
        );

        feedUnlocked =
                false;
    }
}
