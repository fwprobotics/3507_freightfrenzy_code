// package org.firstinspires.ftc.teamcode;

// import org.firstinspires.ftc.teamcode.subsystems.Pipeline;

// import com.qualcomm.hardware.bosch.BNO055IMU;
// import org.firstinspires.ftc.robotcore.external.navigation.Position;
// import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

// import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// import org.firstinspires.ftc.robotcore.external.Telemetry;
// import org.opencv.core.Core;
// import org.opencv.core.Mat;
// import org.opencv.core.Point;
// import org.opencv.core.Rect;
// import org.opencv.core.Scalar;
// import org.opencv.imgproc.Imgproc;
// import org.openftc.easyopencv.OpenCvCamera;
// import org.openftc.easyopencv.OpenCvCameraRotation;
// import org.openftc.easyopencv.OpenCvCameraFactory;
// import org.openftc.easyopencv.OpenCvPipeline;

// import org.firstinspires.ftc.teamcode.subsystems.Arm;

// @Autonomous
// public class PipelineSubsytemTest extends LinearOpMode {
    
//     OpenCvCamera webcam;
//     Pipeline pipeline;
    
//     //Holds analysis
//   CupPosition cupPos;
  
//  //Stores possible analysis results from pipeline   
//      public enum CupPosition {
//         LEFT, // A
//         MIDDLE, // B
//         RIGHT // C
//     }

//     public void runOpMode(){
     
//      //Turns on camera   
//          initCV();
         


//         //
//         waitForStart();
        
//       //Gets analysis from pipeline 
//          findCup();

// //Shows analysis in telemetry (this is mostly a testing thing)
//          telemetry.addData("Position", cupPos);

//          telemetry.update();
//         //

// sleep(10000);
        
        


//     }

// //Function to turn on camera
//  private void initCV() {
//      // Sets variable for the camera id
//         int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//       // Gives a name to the webcam
//         WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//         // Combines the above to create a webcam that we will use
//         webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//         //Sets our pipeline to view images through as the one we want
//         pipeline = new Pipeline(110,180,90,-5);
//         webcam.setPipeline(pipeline);

// // Turns on the webcam
//         webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//         {
//             @Override
//             public void onOpened()
//             {
//                 webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
//             }
//             //This is needed so it knows what to do if something goes wrong
//             public void onError(int thing){
//                 telemetry.addData("error",thing );
//             }
            
//         });

//     }
    
//     private void findCup() {

// //Takes some time so we can run everything through the pipeline
//         sleep(4000);

//         Pipeline.Position cupPosition = pipeline.position;
        
// //Sets cupPos to a corresponding position basesd on pipeline analysis

//         if (cupPosition == Pipeline.Position.LEFT)
//         {
//             cupPos = CupPosition.LEFT;

//         }
//     else if (cupPosition == Pipeline.Position.RIGHT) 
//         {
//             cupPos = CupPosition.RIGHT;

//         }
//       else if (cupPosition == Pipeline.Position.MIDDLE) 
//         {
//           cupPos = CupPosition.MIDDLE;

//         }

//     }
// }