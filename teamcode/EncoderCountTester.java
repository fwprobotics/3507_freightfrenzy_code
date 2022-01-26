package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "EncoderTester", group = "TeleOp")

public class EncoderCountTester extends LinearOpMode {
    private DcMotor backleft;
   
    public void runOpMode() {
         backleft = hardwareMap.dcMotor.get("backLeftDrive");
         backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         waitForStart();
         
    while (opModeIsActive()) {
        telemetry.addData("Encoder count", backleft.getCurrentPosition());
        telemetry.update();
    }
     }

    // todo: write your code here
}