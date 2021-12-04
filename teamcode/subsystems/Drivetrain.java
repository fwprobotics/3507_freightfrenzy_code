package org.firstinspires.ftc.teamcode.subsystems;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    public LinearOpMode l;
    public Telemetry realTelemetry;

    private boolean inputButtonPressed;

    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(RevRobotics20HdHexMotor.class);

    public static class TeleOpDTConstants {
        //Biases so we don't go too fast
        public static double turning_modifier = 0.70;
        public static double y_modifier = 0.80;
        public static double x_modifier = 0.70;
        public static double speedFactor = 0.7;

    }


    public Drivetrain(LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry){

        l = Input;
        realTelemetry = telemetry;
        realTelemetry.setAutoClear(true);

        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        

        l.idle();
    }

    //This is the teleop drive formulas
    public void JoystickMovement(double leftStickY, double leftStickX, double rightStickX, boolean slowModeControl){
        
        double slowModeMult = slowModeControl ? 0.5 : 1;

        //Sets motor values based on adding and subtracting joystick values
        double LeftX = cubeInput(-leftStickX, TeleOpDTConstants.speedFactor);
        double LeftY = cubeInput(-leftStickY, TeleOpDTConstants.speedFactor);
        double RightX = cubeInput(-rightStickX, TeleOpDTConstants.speedFactor);

        double frontLeftVal = ((LeftY - RightX) - LeftX);
        double frontRightVal = ((LeftY + RightX) + LeftX);
        double backLeftVal = ((LeftY - RightX) + LeftX);
        double backRightVal = ((LeftY + RightX) - LeftX);
        
        

        frontLeftDrive.setPower(frontLeftVal * slowModeMult);
        frontRightDrive.setPower(frontRightVal * slowModeMult);
        backLeftDrive.setPower(backLeftVal * slowModeMult);
        backRightDrive.setPower(backRightVal * slowModeMult);
        
        realTelemetry.addData("leftx", LeftX);
        realTelemetry.addData("-left stick x", -leftStickX);
    }
    
    double cubeInput (double input, double factor) {
        double t = factor * Math.pow(input,3 );
        double r = input * (1 - factor);
        return t + r;
        
    }
}
