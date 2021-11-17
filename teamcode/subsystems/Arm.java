package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
Arm subsystem. By Jake, 11/9/20.
 */


public class Arm{

    public DcMotor armMotor;
    public LinearOpMode l;
    public Telemetry realTelemetry;

    public static class ArmConstants {
        public static double arm_modifier = 0.3;
        public static double arm_stall_power = 0.01;
    }

    public enum armRunMode {
        AUTONOMOUS,
        TELEOP
    }

    public Arm(armRunMode runMode, LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry){

        l = Input;

        realTelemetry = telemetry;

        armMotor = hardwareMap.dcMotor.get("armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 

        if (runMode.equals(armRunMode.TELEOP)) {
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        if (runMode.equals(armRunMode.AUTONOMOUS)) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
            armMotor.setTargetPosition(0);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

           
        }

    }

  

    public void teleOpControl(double input, boolean up, boolean down, boolean side){
        
        if (down == true) {
            armMotor.setTargetPosition(150);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            armMotor.setPower(.3);
                
            
        }
          if (up == true) {
            armMotor.setTargetPosition(2250);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            armMotor.setPower(.3);
        }
        if (side == true){
             armMotor.setTargetPosition(790);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            armMotor.setPower(.3);
        }
        if ((down == false) && (up == false) &! armMotor.isBusy()){
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(input * ArmConstants.arm_modifier);
        realTelemetry.addData("Arm power:", input * ArmConstants.arm_modifier);
        realTelemetry.addData("Arm Encoder Count:", armMotor.getCurrentPosition());
        }
    }
    
    public void autoPositions(int position) {
        if (position == 1){
            armMotor.setTargetPosition(290);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            armMotor.setPower(.3);
            
        }
        else if (position == 2){
            armMotor.setTargetPosition(660);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            armMotor.setPower(.3);
            
        }
        else if (position == 3){
            armMotor.setTargetPosition(1150);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            armMotor.setPower(.3);
            
        }
        else if (position == 4){
            armMotor.setTargetPosition(780);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            armMotor.setPower(.3);
            
        }
        else if (position == 5){
            armMotor.setTargetPosition(0);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            armMotor.setPower(.3);
            
        }
       while (armMotor.isBusy()) {
           l.idle();
           
       }


}
}
