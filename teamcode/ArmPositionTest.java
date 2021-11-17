package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
@TeleOp

public class ArmPositionTest extends LinearOpMode{
    
@Override

public void runOpMode() {


boolean a_pushed = false;
boolean b_pushed = false;
boolean x_pushed = false;
boolean y_pushed = false;
    
        telemetry.addLine("Ready and WAITING :)");
        telemetry.update();
        
        Arm arm;

        arm = new Arm(Arm.armRunMode.AUTONOMOUS, this, hardwareMap, telemetry);

        waitForStart();
        telemetry.clearAll();

        if (opModeIsActive()) {

            telemetry.clearAll();
                

            while (opModeIsActive()) {

if (gamepad1.a == true){
    a_pushed = true;
arm.autoPositions(1);
    
} else {
    a_pushed = false;
}

if (gamepad1.b == true){
    b_pushed = true;
arm.autoPositions(2);
    
} else {
    b_pushed = false;
}

if (gamepad1.x == true){
    x_pushed = true;
arm.autoPositions(3);
    
} else {
    x_pushed = false;
}

if (gamepad1.y == true){
    y_pushed = true;
arm.autoPositions(4);
    
} else {
    y_pushed = false;
}


            }
        }
    }
}
