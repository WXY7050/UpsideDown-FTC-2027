package org.firstinspires.ftc.teamcode.upsidedown.subsystems;

import org.firstinspires.ftc.teamcode.upsidedown.RobotConstants;
import org.firstinspires.ftc.teamcode.upsidedown.hardware.RobotHardware;

public class IntakeSubsystem {
    private RobotHardware hardware;

    public IntakeSubsystem(RobotHardware hardware) {
        this.hardware = hardware;
    }

    public void intake() { hardware.intake.setPower(RobotConstants.INTAKE_POWER); }

    public void outtake() { hardware.intake.setPower(RobotConstants.OUTTAKE_POWER); }

    public void stop() { hardware.intake.setPower(0); }

    public void setIntakePower(double power) { hardware.intake.setPower(power); }

}