package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;



public class Lift{

    public DcMotor leftLiftMotor;
    public DcMotor rightLiftMotor; //Not exactly sure what to do with this
    public LinearOpMode l;
    public Telemetry realTelemetry;
    public double basePos
    public double bottomPos;
    public double middlePos;
    public double topPos;
    

    public enum liftRunMode {
        AUTONOMOUS,
        TELEOP
    }
    
    public enum dropoffOptions {
        BASE,
        BOTTOM,
        MIDDLE,
        TOP
    }

    public Lift(liftRunMode runmode, LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry){

        l = Input;
        realTelemetry = telemetry;

        leftLiftMotor = hardwareMap.dcMotor.get("leftLiftMotor");
        rightLiftMotor = hardwareMap.dcMotor.get("rightLiftMotor");

        // Different motor configurations depending on use case
        if (runmode.equals(liftRunMode.AUTONOMOUS)){
            leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Hopefully switch all these to run using encoder
            leftLiftMotor.setTargetPosition(0);
            leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (runmode.equals(liftRunMode.TELEOP)){
            leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); 
            leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            
            rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    public void setPosition(dropoffOptions position){
        switch position {
            case BASE:
                leftLiftMotor.setTargetPosition(basePos); // Probably zero
                break;
            case BOTTOM:
                leftLiftMotor.setTargetPosition(bottomPos);
                break;
            case MIDDLE:
                leftLiftMotor.setTargetPosition(middlePos);
                break;
            case TOP:
                leftLiftMotor.setTargetPosition(topPos);
                break;
        }
        leftLiftMotor.setPower(1); //1 seems like a lot but we'll see
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void teleOpControl(double input, boolean up, boolean mid, boolean down, boolean base){ // Rename inputs based on real buttons we choose
       if down {
            leftLiftMotor.setTargetPosition(bottomPos);
            leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            leftLiftMotor.setPower(1);
                
            
        }
          if up {
            armMotor.setTargetPosition(topPos);
            leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            leftLiftMotor.setPower(1);
        }
        if mid{
            armMotor.setTargetPosition(middlePos);
            leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            leftLiftMotor.setPower(1);
        }
        
        if base {
            armMotor.setTargetPosition(basePos);
            leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            leftLiftMotor.setPower(1);
        }
        if (!down &! up &! mid &! base &! armMotor.isBusy()){
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setPower(input); //Probably should/can get toned down
        }
    }


}
