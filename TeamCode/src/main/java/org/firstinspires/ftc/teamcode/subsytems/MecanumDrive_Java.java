package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Arm{

    public DcMotor armMotor;
    public LinearOpMode l;
    public Telemetry realTelemetry;

  public void teleOpControl(double input){
    private void armControl() {
        if (gamepad2.x) {
          armPos = "down";
        } 

        else if (gamepad2.y) {
          armPos = "up";
        }

        // Bringing the arm up
        if (armPos == "up" && armPot.getVoltage() >= topArmPosPot + potMargin) {
          armMotor.setPower(0.7);

          // if (clawPos == "closed") {
          //   armMotor.setPower(-0.8);
          // }

          // else if (clawPos == "open") {
          //   armMotor.setPower(-0.3);
          // }
        }

        // Bringing the arm down
        else if (armPos == "down" && armPot.getVoltage() <= bottomArmPosPot - potMargin) {
          armMotor.setPower(-0.2);
        }

        else {
          armMotor.setPower(0);
        }
      
      
      //Jake's code from last year :)
        //armMotor.setPower(input * ArmConstants.arm_modifier);
        //realTelemetry.addData("Arm power:", input * ArmConstants.arm_modifier);
        //realTelemetry.addData("Arm Encoder Count:", armMotor.getCurrentPosition());
    }
      }
