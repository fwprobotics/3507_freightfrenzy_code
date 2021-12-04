package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Arm{

    public DcMotorEx armMotor;
    public LinearOpMode l;
    public Telemetry realTelemetry;

    public static class ArmConstants {
        //We only run at 30% power so nothing crazy happens (this maybe should be increased though)
        public static double arm_modifier = 0.7;
        public static double arm_stall_power = 0.01;
        public static double arm_p = 15.0;
        public static double arm_i = 0.1;
        public static double arm_d = 0.1;
        public static double arm_f = 0.00;
    }

    public enum armRunMode {
        AUTONOMOUS,
        TELEOP
    }
    
    public enum autoOptions {
        GROUND,
        BOTTOM,
        MIDDLE,
        OVERSHOOT,
        TOP
    }
    
 

    public Arm(armRunMode runMode, LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry){

        l = Input;

        realTelemetry = telemetry;

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        

        if (runMode.equals(armRunMode.TELEOP)) {
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            realTelemetry.addData("PID Coefficients", armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

        }

        if (runMode.equals(armRunMode.AUTONOMOUS)) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
            armMotor.setTargetPosition(0);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

           
        }

    

   // public void PIDSet() {
      armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
         ArmConstants.arm_p, ArmConstants.arm_i, ArmConstants.arm_d, ArmConstants.arm_f)
        );
   // }
}

   // PIDSet();
  

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
        realTelemetry.addData("PID Coefficients", armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        }
    }
    
    
    public void autoPositions(autoOptions position) {
        
        switch(position){
            case BOTTOM:
                armMotor.setTargetPosition(290);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
                armMotor.setPower(.3);
                break;
            case MIDDLE:
                armMotor.setTargetPosition(660);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
                armMotor.setPower(.3);
                break;
            case OVERSHOOT:
                armMotor.setTargetPosition(1150);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
                armMotor.setPower(.3);
            case TOP:
                armMotor.setTargetPosition(780);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
                armMotor.setPower(.3);
                break;
            case GROUND:
                armMotor.setTargetPosition(0);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
                armMotor.setPower(.3);
                break;
        }
        
        /*Low drop off position
        if (position == 1){
            armMotor.setTargetPosition(290);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            armMotor.setPower(.3);
            
        }
        // Middle drop off position
        else if (position == 2){
            armMotor.setTargetPosition(660);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            armMotor.setPower(.3);
            
        }
        //Above the top level
        else if (position == 3){
            armMotor.setTargetPosition(1150);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            armMotor.setPower(.3);
            
        }
        // At the top level (the idea is to go from 3 to here)
        else if (position == 4){
            armMotor.setTargetPosition(780);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            armMotor.setPower(.3);
            
        }
        // All the way down (do this to make sure there is no funny stuff with the encoder in teleop)
        else if (position == 5){
            armMotor.setTargetPosition(0);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            armMotor.setPower(.3);
            
        }
        */
        //Makes sure the robot is not trying to do stuff while the arm is moving
       while (armMotor.isBusy()) {
           l.idle();
           
       }


}
}
