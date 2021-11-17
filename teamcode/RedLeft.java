package org.firstinspires.ftc.teamcode;

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
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

@Autonomous(name="RedLeft", group="chad")
public class RedLeft extends LinearOpMode {
    
      OpenCvCamera webcam;
    WhichPosition pipeline;
    
    //Holds analysis
   CupPosition cupPos;
    //
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    Arm arm;
    
    //28 * 20 / (2ppi * 4.125)
    Double width = 12.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 40;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 1.075;//default 0.8
    Double meccyBias = 1.189;//change to adjust only strafing movement
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

    public void runOpMode(){
        //
        initGyro();
        //
        frontleft = hardwareMap.dcMotor.get("frontLeftDrive");
        frontright = hardwareMap.dcMotor.get("frontRightDrive");
        backleft = hardwareMap.dcMotor.get("backLeftDrive");
        backright = hardwareMap.dcMotor.get("backRightDrive");
        
      arm = new Arm(Arm.armRunMode.AUTONOMOUS, this, hardwareMap, telemetry);


        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE); 
     
     //Turns on camera   
         initCV();
         


        //
        waitForStart();
        
       //Gets analysis from pipeline 
         findCup();

//Shows analysis in telemetry (this is mostly a testing thing)
         telemetry.addData("Position", cupPos);

         telemetry.update();
        //



//Moves arm motor to a certain position based on where the cup is 

//if (cupPos == CupPosition.LEFT) {
// arm.autoPositions(1);
//} else if (cupPos == CupPosition.MIDDLE) {
// arm.autoPositions(2);    
//} else if (cupPos == CupPosition.RIGHT) {
// arm.autoPositions(3);    
//}

//


        
        



//Motion via CHAD
         //
        
    moveToPosition(1, 0.3);
    //
    strafeToPosition(-10.0, 0.3);
    //
    moveToPosition(25.5, 0.3);
    //
    turnWithGyro(90, 0.2);
    //
    moveToPosition(18, 0.2);
    //
    sleep(1500);
    //
    moveToPosition(-18, 0.3);
    //
    strafeToPosition(15.5, 0.3);
    //
    moveToPosition(8, 0.3);
    //
    strafeToPosition(13.5, 0.3);
    //
    moveToPosition(46, 0.7);
}
    
    //Image pipeline
    public static class WhichPosition extends OpenCvPipeline {
    
    //Defines possible position outcomes
    public enum Position
    {
        LEFT,
        MIDDLE,
        RIGHT
    }

        //Defines dimensions for the boxes
        final int REGION_WIDTH = 25;
        final int REGION_HEIGHT = 30;

        //Defines starting points for the first box
        public static int box1x = 105;
        public static int box1y = 180;

        //Defines a Mat specifically for the first box
        Mat box1_Hue;

        //Defines an int to take the hue of the first box
        int box1_average_hue;

        

        //Defines an anchor point for the first box using the dimensions
        final Point BOX1_TOPLEFT_ANCHOR_POINT = new Point(box1x, box1y);
            
        //Defines the two points used to draw the first box
        Point box1_pointA = new Point(
            BOX1_TOPLEFT_ANCHOR_POINT.x,
            BOX1_TOPLEFT_ANCHOR_POINT.y);
        Point box1_pointB = new Point(
            BOX1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            BOX1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        //We repeat our steps for the second box
        public static int box2x = 195;
        public static int box2y = 175;

        Mat box2_Hue;

        int box2_average_hue;

        final Point BOX2_TOPLEFT_ANCHOR_POINT = new Point(box2x, box2y);
            
        Point box2_pointA = new Point(
            BOX2_TOPLEFT_ANCHOR_POINT.x,
            BOX2_TOPLEFT_ANCHOR_POINT.y);
        Point box2_pointB = new Point(
            BOX2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            BOX2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            
// And the third
        public static int box3x = 285;
        public static int box3y = 170;
        //

        Mat box3_Hue;

        int box3_average_hue;

        final Point BOX3_TOPLEFT_ANCHOR_POINT = new Point(box3x, box3y);
            
        Point box3_pointA = new Point(
            BOX3_TOPLEFT_ANCHOR_POINT.x,
            BOX3_TOPLEFT_ANCHOR_POINT.y);
        Point box3_pointB = new Point(
            BOX3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            BOX3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        
        // Sets ideal hue
        int HueGoal = 70;

        // ints to hold distance from hue goal
        int box1_deviation;
        int box2_deviation;
        int box3_deviation;
    
        //Defines colors to draw boxes with
        final Scalar RED = new Scalar(225, 0, 0);
        
        final Scalar WHITE = new Scalar(225, 225, 225);

        final Scalar BLACK = new Scalar(0,0,0);

        // Sets up a variable to store our analysis and sets it to a default (what this is doesn't matter)
        private volatile Position position = Position.MIDDLE;  


// Actually does the image processing  
    @Override
    public Mat processFrame(Mat inputMat) {
        //Converts to HSV
        Imgproc.cvtColor(inputMat, inputMat, Imgproc.COLOR_RGB2HSV);

            //Sets the box1 mat to the stuff in box1
            box1_Hue = inputMat.submat(new Rect(box1_pointA, box1_pointB));

            //Takes the average hue of box1
            box1_average_hue = (int) Core.mean(box1_Hue).val[0];

             //Repeats for box 2
             box2_Hue = inputMat.submat(new Rect(box2_pointA, box2_pointB));

             box2_average_hue = (int) Core.mean(box2_Hue).val[0];

               //And box 3
               box3_Hue = inputMat.submat(new Rect(box3_pointA, box3_pointB));

               box3_average_hue = (int) Core.mean(box3_Hue).val[0];

// Sets deviation from the distance of the average hue to the ideal
               box1_deviation = Math.abs(box1_average_hue - HueGoal);
               box2_deviation = Math.abs(box2_average_hue - HueGoal);
               box3_deviation = Math.abs(box3_average_hue - HueGoal);
    
            //Checks which box has the least deviation from the ideal hue, and gives that as the box with the element in it
            if (box1_deviation < box2_deviation && box1_deviation<box3_deviation) {
                position = Position.LEFT;
            } else if (box2_deviation < box3_deviation) {
                position = Position.MIDDLE;
            } else {
                    position = Position.RIGHT;
                }
                
           
            //Draws box1
            Imgproc.rectangle(
                inputMat, // What to draw on
                box1_pointA, // First point which defines the rectangle
                box1_pointB, // Second point which defines the rectangle
                RED, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

                //Draws box2
            Imgproc.rectangle(
                inputMat, // What to draw on
                box2_pointA, // First point which defines the rectangle
                box2_pointB, // Second point which defines the rectangle
                WHITE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines
                            

                             //And box 3
                             Imgproc.rectangle(
                                inputMat, // What to draw on
                                box3_pointA, // First point which defines the rectangle
                                box3_pointB, // Second point which defines the rectangle
                                BLACK, // The color the rectangle is drawn in
                                2); // Thickness of the rectangle lines

        return inputMat;
    }
    
}

//Function to turn on camera
 private void initCV() {
     // Sets variable for the camera id
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      // Gives a name to the webcam
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Combines the above to create a webcam that we will use
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //Sets our pipeline to view images through as the one we want
        pipeline = new WhichPosition();
        webcam.setPipeline(pipeline);

// Turns on the webcam
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
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
        sleep(4000);

        WhichPosition.Position cupPosition = pipeline.position;
        
//Sets cupPos to a corresponding position basesd on pipeline analysis

        if (cupPosition == WhichPosition.Position.LEFT)
        {
            cupPos = CupPosition.LEFT;

        }
    else if (cupPosition == WhichPosition.Position.RIGHT) 
        {
            cupPos = CupPosition.RIGHT;

        }
      else if (cupPosition == WhichPosition.Position.MIDDLE) 
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
