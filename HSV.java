import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class HSV extends OpenCvPipeline {
    //Defines starting points for the box
    public static int x = 95;
    public static int y = 0;

    //Sets up telemetry (a bit finnicky)
    Telemetry telemetry;

    public HSV(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    
  
    @Override
    public Mat processFrame(Mat inputMat) {
        //Converts to HSV
        Imgproc.cvtColor(inputMat, inputMat, Imgproc.COLOR_RGB2HSV);

        //Defines a Mat specifically for our region
        Mat region1_Hue;

        //Defines an float to take the hue of the rectangle
        int rec_average_hue;

        //Defines a string that we will later use to say if we see a cup.
        String Cup = "Maybe"; 

        //Sets bounds on the hue (you can add parameters for the other stuff)
        int HueMin = 53;
        int HueMax = 75;

        //Defines red color to draw box with
         final Scalar RED = new Scalar(225, 0, 0);

         //Defines dimensions for the box
         final int REGION_WIDTH = 40;
         final int REGION_HEIGHT = 75;

         //Defines an anchor point for the box using the dimensions
         final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(x, y);

         //Defines the two points used to draw the box
         Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
         Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

            //Sets region mat to the stuff in the rectangle
            region1_Hue = inputMat.submat(new Rect(region1_pointA, region1_pointB));

            //Takes the average hue of the rectangle
            rec_average_hue = (int) Core.mean(region1_Hue).val[0];

            
            //Checks if the average hue in the rectangle is between the max and min
            if (HueMin < rec_average_hue && rec_average_hue < HueMax) {
                Cup = "Yes";
            } else {
                    Cup = "No";
                }
                telemetry.addData("Cup", Cup);
            telemetry.update();
           
           


            //Draws the box
            Imgproc.rectangle(
                inputMat, // What to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                RED, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines
                
              
              
        //Shows the image
        return inputMat;
    }
    
}
