package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
// import org.firstinspires.ftc.teamcode.subsystems.Arm;


@TeleOp(name = "MecanumDrive", group = "TeleOp")
public class MecanumDrive extends LinearOpMode {
    private Servo clawServo;
        


    Drivetrain drivetrain;
    Arm arm;
    Claw claw;
    

    @Override
    public void runOpMode() {
        
        drivetrain = new Drivetrain(this, hardwareMap, telemetry);
        arm = new Arm(Arm.armRunMode.TELEOP, this, hardwareMap, telemetry);
        claw = new Claw(hardwareMap);
        
        telemetry.addLine("Ready and WAITING :)");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        if (opModeIsActive()) {

            telemetry.clearAll();

            while (opModeIsActive()) {

                drivetrain.JoystickMovement(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_bumper);
                arm.teleOpControl(gamepad2.right_stick_y, gamepad2.dpad_up ,gamepad2.dpad_down,gamepad2.dpad_left,gamepad2.y);
                /* if (gamepad2.x || gamepad1.x) {
                    Claw.closeClaw();} // FIXME this function should take a button as its input like arm.teleOpControl
                if (gamepad2.y || gamepad1.y) {
                    Claw.openClaw();} // FIXME same as above
                    telemetry.update();
                    */
                claw.TeleopControl(gamepad2.x || gamepad1.x);
                
            }
        }
    }
}
