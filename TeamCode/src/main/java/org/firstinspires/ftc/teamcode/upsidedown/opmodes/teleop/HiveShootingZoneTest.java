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
@TeleOp(name = "Hive Shooting Zone Test", group = "Test")
public class HiveShootingZoneTest extends OpMode {


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
    // 暂时测试扇区
    //
    // 这个 ±20° 只是测试值
    // 后面我们通过实测重新确定
    // ============================================================

    private static final double TEST_HALF_ANGLE = 20.0;


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


        boolean insideSector =
                Math.abs(
                        sectorError
                )
                        <= TEST_HALF_ANGLE;


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
                "In Sector",
                insideSector
                        ? "YES"
                        : "NO"
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
                "InSector",
                insideSector ? 1 : 0
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