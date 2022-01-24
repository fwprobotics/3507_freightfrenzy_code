package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Intake {

    private DcMotor intakeMotor;
    private LinearOpMode l;
    private Telemetry realTelemetry;

    public enum intakeStatuses {
        ON,
        OFF
    }

    public enum intakeDirections {
        FORWARD,
        REVERSE
    }

    private intakeStatuses intakeStatus = intakeStatuses.OFF;
    public intakeDirections intakeDirection = intakeDirections.FORWARD;
    private boolean inputButtonPressed;
    private int direction = -1;

    public static class IntakeConstants {
        public static double intake_power = 1.0;

    }


    public Intake(LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry){

        l = Input;
        realTelemetry = telemetry;
        realTelemetry.setAutoClear(true);

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    // TELEOP FUNCTIONS

    // Toggle on and off intake via inputButton
    public void toggleIntake(boolean inputButton){
        if (inputButton && !inputButtonPressed) {
            switch (intakeStatus) {
                case OFF:
                    inputButtonPressed = true;
                    intakeStatus = intakeStatuses.ON;
                    break;
                case ON:
                    inputButtonPressed = true;
                    intakeStatus = intakeStatuses.OFF;
                    break;
            }
        }

        if (!inputButton) {
            inputButtonPressed = false;
        }
    }

    // Sets power of intake depending on direction
    public void runIntake(){
        if (intakeStatus == intakeStatuses.ON) {
            intakeMotor.setPower(IntakeConstants.intake_power * direction);

        }
        else if (intakeStatus == intakeStatuses.OFF) {
            intakeMotor.setPower(0);

        }
    }

    // While input button is held down intake is reversed
    public void directionControl(boolean inputButton) {
        if (!inputButton) {
            direction = 1;
            intakeDirection = intakeDirections.FORWARD;
        } else {
            direction = -1;
            intakeDirection = intakeDirections.REVERSE;
        }

    }

}
