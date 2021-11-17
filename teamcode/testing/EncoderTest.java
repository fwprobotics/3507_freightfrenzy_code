package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "Testing")
public class EncoderTest extends LinearOpMode {

    private DcMotor backleftEncoder, backrightEncoder, frontleftEncoder, frontrightEncoder;

    @Override
    public void runOpMode() {
        
      

        backleftEncoder = hardwareMap.dcMotor.get("backLeftDrive");
        backrightEncoder = hardwareMap.dcMotor.get("backRightDrive");
        frontleftEncoder = hardwareMap.dcMotor.get("frontLeftDrive");
        frontrightEncoder = hardwareMap.dcMotor.get("frontRightDrive");

        telemetry.addLine("Ready and WAITING");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        backleftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (opModeIsActive()) {

            while (opModeIsActive()) {

                telemetry.addData("Back Left Encoder:", backleftEncoder.getCurrentPosition());
                telemetry.addData("Back Right Encoder:", backrightEncoder.getCurrentPosition());
                telemetry.addData("Front Left Encoder:", frontleftEncoder.getCurrentPosition());
                telemetry.addData("Front Right Encoder:", frontrightEncoder.getCurrentPosition());

                telemetry.update();

            }
        }
    }
}
