//package org.firstinspires.ftc.teamcode.upsidedown.opmodes.auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.upsidedown.hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.upsidedown.subsystems.DriveSubsystem;
//import org.firstinspires.ftc.teamcode.upsidedown.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.upsidedown.subsystems.LauncherSubsystem;
//
//@Autonomous(name = "UpsideDown Auto Red Left", group = "UpsideDown")
//public class UpsideDownAutoRedLeft extends LinearOpMode {
//    private RobotHardware robot;
//    private DriveSubsystem drive;
//    private IntakeSubsystem intake;
//    private LauncherSubsystem launcher;
//    private ElapsedTime runtime = new ElapsedTime();
//
//    @Override
//    public void runOpMode() {
//        robot = new RobotHardware();
//        robot.init(hardwareMap);
//
//        drive = new DriveSubsystem(robot);
//        intake = new IntakeSubsystem(robot);
//        launcher = new LauncherSubsystem(robot);
//
//        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        telemetry.addData("状态", "初始化完成");
//        telemetry.addData("联盟", "红方");
//        telemetry.addData("位置", "左侧");
//        telemetry.addData("", "按下 Start 开始");
//        telemetry.update();
//
//        waitForStart();
//        robot.resetIMU();
//        runtime.reset();
//
//        if (opModeIsActive()) {
//            // ========== 自动程序序列 ==========
//
//            // 步骤1: 准备发射系统
//            telemetry.addData("步骤", "1. 准备发射");
//            telemetry.update();
//            launcher.setAngleMid();
//            launcher.spinUp();
//            sleep(1500); // 等待发射轮加速
//
//            // 步骤2: 发射预装载样本
//            telemetry.addData("步骤", "2. 发射样本");
//            telemetry.update();
//            if (launcher.isAtSpeed()) {
//                launcher.fire();
//                sleep(500);
//                launcher.resetTrigger();
//            }
//            sleep(500);
//            launcher.stop();
//
//            // 步骤3: 前进到区域
//            telemetry.addData("步骤", "3. 前进");
//            telemetry.update();
//            driveForward(0.5, 1000);
//            stopAndWait(200);
//
//            // 步骤4: 横移调整位置
//            telemetry.addData("步骤", "4. 横移");
//            telemetry.update();
//            strafeRight(0.5, 500);
//            stopAndWait(200);
//
//            // 步骤5: 展开吸取装置
//            telemetry.addData("步骤", "5. 展开吸取");
//            telemetry.update();
//            intake.deploy();
//            sleep(500);
//
//            // 步骤6: 吸取样本
//            telemetry.addData("步骤", "6. 吸取样本");
//            telemetry.update();
//            intake.intake();
//            driveForward(0.3, 800);
//            sleep(500);
//            intake.stop();
//
//            // 步骤7: 返回并停车
//            telemetry.addData("步骤", "7. 停车");
//            telemetry.update();
//            driveBackward(0.5, 500);
//            stopAndWait(200);
//
//            // 收回系统
//            intake.retract();
//
//            telemetry.addData("状态", "自动程序完成");
//            telemetry.addData("用时", "%.1f 秒", runtime.seconds());
//            telemetry.update();
//        }
//    }
//
//    private void driveForward(double power, long milliseconds) {
//        drive.driveRobotCentric(power, 0, 0);
//        sleep(milliseconds);
//    }
//
//    private void driveBackward(double power, long milliseconds) {
//        drive.driveRobotCentric(-power, 0, 0);
//        sleep(milliseconds);
//    }
//
//    private void strafeLeft(double power, long milliseconds) {
//        drive.driveRobotCentric(0, -power, 0);
//        sleep(milliseconds);
//    }
//
//    private void strafeRight(double power, long milliseconds) {
//        drive.driveRobotCentric(0, power, 0);
//        sleep(milliseconds);
//    }
//
//    private void rotate(double power, long milliseconds) {
//        drive.driveRobotCentric(0, 0, power);
//        sleep(milliseconds);
//    }
//
//    private void stopAndWait(long milliseconds) {
//        drive.stop();
//        sleep(milliseconds);
//    }
//}