package org.firstinspires.ftc.teamcode.Robot.GamepadControllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;
import org.firstinspires.ftc.teamcode.Utils.Vector;

public class BuruDriveTrainControl implements IRobotModule {

    private final Gamepad gamepad;
    private final StickyGamepad stickyGamepad;
    private final MecanumDrive drive;

    public BuruDriveTrainControl(Gamepad gamepad, MecanumDrive drive){
        this.gamepad = gamepad;
        this.stickyGamepad = new StickyGamepad(gamepad);
        this.drive = drive;
    }

    @Override
    public void update() {
//        if(gamepad.b) drive.overallMultiplier = MecanumDrive.slowMultiplier;
//        else drive.overallMultiplier = MecanumDrive.fastMultiplier;

        if(gamepad.left_bumper && gamepad.right_bumper) {
            drive.getLocalizer().imu.resetImu();
        }

        drive.setTargetVector(new Vector(-gamepad.left_stick_y, -gamepad.left_stick_x, gamepad.left_trigger - gamepad.right_trigger));

        stickyGamepad.update();
    }
}
