package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

//This is quite self explanatory - just moves a servo between positions based on if we want the claw open or not
public class Claw {
    public static Servo ClawServo;
    
    public Claw(HardwareMap hardwareMap){
        ClawServo = hardwareMap.servo.get("clawServo");
    }
    
    boolean toggle = false;
    boolean pressed;
    
    public void TeleopControl(boolean press){
        
        if (press) {
            if (!pressed){
                if (!toggle){
                    toggle = true;
                } else {
                    toggle = false;
                }
            }
            pressed = true;
        } else {
            pressed = false;
        }
        
        if (!toggle) {
            closeClaw();
        } else {
            openClaw();
        }
    }
    
    public static void closeClaw() {
              ClawServo.setPosition(1);
    }
       public static void openClaw() {
              ClawServo.setPosition(0.6);
    }

}