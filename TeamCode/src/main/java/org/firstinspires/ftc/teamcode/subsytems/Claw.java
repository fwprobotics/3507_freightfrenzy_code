package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    public static Servo ClawServo;
    
    public Claw(HardwareMap hardwareMap){
        ClawServo = hardwareMap.servo.get("clawServo");
    }
    
    public static void closeClaw() {
              ClawServo.setPosition(0.25);
    }
       public static void openClaw() {
              ClawServo.setPosition(0.5);
    }

    // todo: write your code here
}
