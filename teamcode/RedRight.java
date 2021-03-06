package org.firstinspires.ftc.teamcode;

//starting imports
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Pipeline;
import org.firstinspires.ftc.teamcode.subsystems.LoopyPipeline;

// start of code
@Autonomous(name="RedRight", group="chad")
public class RedRight extends LinearOpMode {
    
    OpenCvCamera webcam;
    LoopyPipeline pipeline;
    
    //Holds analysis
   CupPosition cupPos;
    
// defines variables for motors and servos
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    Arm arm;
    Claw claw;
    
    //28 * 20 / (2ppi * 4.125)
    Double width = 12.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 40;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    // 1 CHAD tick should be 1 inch but this might not actually be the case
    Double bias = .717; //default 0.8
    Double meccyBias = .795; //change to adjust only strafing movement
    //
    Double conversion = cpi * bias;
    Boolean exit = false;
    //
    
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    //
 
 //Stores possible analysis results from pipeline   
     public enum CupPosition {
        LEFT, // A
        MIDDLE, // B
        RIGHT // C
    }
// initializing and running autonomous
    public void runOpMode(){
        //
        initGyro();
        
        // sets variables for each motor and servo
        frontleft = hardwareMap.dcMotor.get("frontLeftDrive");
        frontright = hardwareMap.dcMotor.get("frontRightDrive");
        backleft = hardwareMap.dcMotor.get("backLeftDrive");
        backright = hardwareMap.dcMotor.get("backRightDrive");
        
      arm = new Arm(Arm.armRunMode.AUTONOMOUS, this, hardwareMap, telemetry);
      claw = new Claw(hardwareMap);

// reverses the direction of left wheels
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE); 
     
     //Turns on camera   
         initCV();
         telemetry.addLine("Ready and waiting!");
         telemetry.update();


        //
        waitForStart();
        
        //
        //
        // BEGIN MOVEMENT
        //
        //
        
        //Grasps starting block 
        claw.closeClaw();  
        
        //Gets analysis from pipeline 
        findCup();

        //Shows above analysis in telemetry (this is mostly a testing thing)
        telemetry.addData("Position", cupPos);
        telemetry.addData("Box x", pipeline.box1xfinal);
        telemetry.addData("Box y", pipeline.box1yfinal);
        telemetry.addData("Deviation", pipeline.min_deviation);
        telemetry.addData("Saturation", pipeline.finalsat);
        telemetry.addData("Hue", pipeline.besthue);
        telemetry.update();

        //Moves arm motor to the position that corresponds to the cup position 
        switch(cupPos) {
            case LEFT:  
                arm.autoPositions(Arm.positionOption.BOTTOM);
                break;
            case MIDDLE:  
                arm.autoPositions(Arm.positionOption.MIDDLE);
                break;
            case RIGHT: 
                arm.autoPositions(Arm.positionOption.OVERSHOOT);
                break;
        }
        telemetry.update();


        //Motion via CHAD
        //

        moveToPosition(1.5, 0.2);
        //
        //turn off camera and strafes 
        webcam.stopStreaming();
        webcam.closeCameraDevice();
        strafeToPosition(-17.5, 0.2);
        //

        //based on cup position, moves to a certain position closer or further from Alliance shipping hub, lets go of 
        if (cupPos == CupPosition.LEFT) {
            moveToPosition(21, 0.2);
            claw.openClaw();
            sleep(500);
            claw.closeClaw();
            moveToPosition(-6, 0.2);
            turnWithGyro(90, 0.2);
        } else if (cupPos == CupPosition.MIDDLE) {
            moveToPosition(22.25, 0.2);
            claw.openClaw();
            sleep(500);
            claw.closeClaw();
            moveToPosition(-6, 0.2);
            turnWithGyro(90,.2);
        } else if (cupPos == CupPosition.RIGHT) {
            moveToPosition(24.8, 0.2);
         /*   turnWithGyro(180, 0.2);
            sleep(750);
            arm.autoPositions(3);
            moveToPosition(-7, 0.3);
                  moveToPosition(-7.2, 0.1);
*/
            arm.autoPositions(Arm.positionOption.TOP);
            claw.openClaw();
            sleep(1000);
            arm.autoPositions(Arm.positionOption.OVERSHOOT);
            sleep(500);
            claw.closeClaw();
            moveToPosition(-3, 0.2);
            turnWithGyro(90, 0.2);
        }
        //

        //
        strafeToPosition(25, 0.4);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setPower(0.8);
        frontright.setPower(0.75);
        backleft.setPower(0.8);
        backright.setPower(0.75);
        //
        sleep(2000);
        strafeToPosition(-30, 0.5);
        //
        arm.autoPositions(Arm.positionOption.GROUND);
    }
    
 

//Function to turn on camera and me ;)
 private void initCV() {
     // Sets variable for the camera id
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      // Gives a name to the webcam
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Combines the above to create a webcam that we will use
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //(Boundary between regions 1 and 2, Boundary between 2 and 3, Far left, Far top, Far right, Far bottom, opmode, the side we're on)
        pipeline = new LoopyPipeline(175, 240, 90, 130, 290, 145, this, LoopyPipeline.Side.RED);
        webcam.setPipeline(pipeline);

// Turns on the webcam
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //Disable the below during a tournament
               webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
            //This is needed so it knows what to do if something goes wrong
            public void onError(int thing){
                telemetry.addData("error",thing );
            }
            
        });

    }
    
    
     private void findCup() {

        //Takes some time so we can run everything through the pipeline
        sleep(3500);

        LoopyPipeline.Position cupPosition = pipeline.position;
        
        //Sets cupPos to a corresponding position basesd on pipeline analysis

        if (cupPosition == LoopyPipeline.Position.LEFT)
        {
            cupPos = CupPosition.LEFT;

        }
       else if (cupPosition == LoopyPipeline.Position.RIGHT) 
        {
            cupPos = CupPosition.RIGHT;

        }
        else if (cupPosition == LoopyPipeline.Position.MIDDLE) 
        {
           cupPos = CupPosition.MIDDLE;

        }

    }


    //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
    public void moveToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches*conversion));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){
            if (exit){
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);
                return;
            }
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }
    //
    /*
    This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
    Degrees should always be positive, make speedDirection negative to turn left.
     */
    public void turnWithGyro(double degrees, double speedDirection){
        //<editor-fold desc="Initialize">
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0){//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            //</editor-fold>
        }else{
            //<editor-fold desc="turn left">
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        //
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }
        //</editor-fold>
        //
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches * cpi * meccyBias));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){}
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }

    //
    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }
    //
    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn. Hi Leena :) 101
     */
    public void turnWithEncoder(double input){
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontleft.setPower(input);
        backleft.setPower(input);
        frontright.setPower(-input);
        backright.setPower(-input);
    }
    //
}
