import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class WhichPosition extends OpenCvPipeline {
    
    //Defines possible position outcomes
    public enum Position
    {
        LEFT,
        MIDDLE,
        RIGHT
    }

        //Defines dimensions for all boxes
        final int REGION_WIDTH = 25;
        final int REGION_HEIGHT = 40;

        //Defines starting points for the first box
        public static int box1x = 5;
        public static int box1y = 165;

        //Defines a Mat specifically for the first box
        Mat box1_Hue;

        //Defines an float to take the hue of the first box
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

        //Defines starting points for the second box
        public static int box2x = 75;
        public static int box2y = 160;

        //Defines a Mat specifically for the second box
        Mat box2_Hue;

        //Defines an float to take the hue of the second box
        int box2_average_hue;

        //Defines an anchor point for the second box using the dimensions
        final Point BOX2_TOPLEFT_ANCHOR_POINT = new Point(box2x, box2y);
            
        //Defines the two points used to draw the second box
        Point box2_pointA = new Point(
            BOX2_TOPLEFT_ANCHOR_POINT.x,
            BOX2_TOPLEFT_ANCHOR_POINT.y);
        Point box2_pointB = new Point(
            BOX2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            BOX2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        
        //Sets bounds on the hue (you can add parameters for the other stuff)
        int HueMin = 53;
        int HueMax = 75;
    
        //Defines red color to draw box with
        final Scalar RED = new Scalar(225, 0, 0);

        // Sets up a variable to store our analysis
        private volatile Position position = Position.RIGHT;           

    //Sets up telemetry (a bit finnicky)
    Telemetry telemetry;

    public WhichPosition(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
  
    @Override
    public Mat processFrame(Mat inputMat) {
        //Converts to HSV
        Imgproc.cvtColor(inputMat, inputMat, Imgproc.COLOR_RGB2HSV);

            //Sets the box1 mat to the stuff in box1
            box1_Hue = inputMat.submat(new Rect(box1_pointA, box1_pointB));

            //Takes the average hue of box1
            box1_average_hue = (int) Core.mean(box1_Hue).val[0];

             //Sets the box2 mat to the stuff in the box2
             box2_Hue = inputMat.submat(new Rect(box2_pointA, box2_pointB));

             //Takes the average hue of box2
             box2_average_hue = (int) Core.mean(box2_Hue).val[0];
    
            //Checks if the average hue in each rectangle is between the max and min
            if (HueMin < box1_average_hue && box1_average_hue < HueMax) {
                position = Position.LEFT;
            } else if (HueMin < box2_average_hue && box2_average_hue < HueMax) {
                position = Position.MIDDLE;
            } else {
                    position = Position.RIGHT;
                }
                telemetry.addData("Position", position);
            telemetry.update();
           
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
                RED, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines
                             
        //Shows the image
        return inputMat;
    }
    
}
