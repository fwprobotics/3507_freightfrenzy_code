package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline; 
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class LoopyPipeline extends OpenCvPipeline {
    
    //Defines possible position outcomes
    public enum Position
    {
        LEFT,
        MIDDLE,
        RIGHT
    }
    
    //Defines possible sides that we can start on
    public enum Side {
        RED,
        BLUE
    }
    
        Side side;

        //Defines dimensions for the boxes
        final int REGION_WIDTH = 10;
        final int REGION_HEIGHT = 25;
        
        final int TAPE_WIDTH = 15;
        final int TAPE_HEIGHT = 5;
 
        public LinearOpMode l;
        
        //Defines starting points for the first box
        public int box1x;
        public int box1y;
        public int box2x;
        public int box2y;
        public int box3x;
        public int box3y;
        int startx;
        int starty;
        int maxx;
        int maxy;
        int sat_threshold = 30;

        //Defines a Mat specifically for the first box
        Mat box1_Mat;

        //Defines an int to take the hue of the first box
        int box1_average_hue;

        //And one for saturation
        int box1_average_sat;
        
        Mat box2_Mat;

        int box2_average_hue;
        
        int box2_average_sat;

        Mat box3_Mat;

        int box3_average_hue;

        int box3_average_sat;


       

        // Sets ideal hue
        int HueGoal = 110;
        int TapeGoal;

        // int to hold distance from hue goal
        int box1_deviation;
        int box2_deviation;
        int box3_deviation;

        // int to hold current deviation
        public int min_deviation;
        public int min_deviation2;
        public int min_deviation3;

        //ints to hold final information
        public int box1xfinal;
        public int box1yfinal;
        public int finalsat;
        
        int box2xfinal;
        int box2yfinal;

        int box3xfinal;
        int box3yfinal;
        
        int box4x = 55;
        int box4y = 157;
        Mat box4_Mat;

        int box4_average_hue;

        int box4_average_sat;
        

        //Sets ints for lines
        int line1x;
        int line2x;
        
        public int besthue;

 

        // Sets up a variable to store our analysis
        public volatile Position position = Position.MIDDLE;  

          //Defines colors to draw box with
        final Scalar RED = new Scalar(225, 0, 0);
        final Scalar BLACK = new Scalar(0, 0, 0);
        final Scalar WHITE = new Scalar(225, 225, 225);
          
          public LoopyPipeline(int x1, int x2, int minx, int miny, int Maxx, int Maxy, LinearOpMode input, Side ourSide){
            line1x = x1;
            line2x = x2;
            startx = minx;
            starty = miny;
            maxx = Maxx;
            maxy = Maxy;
            l = input;
            side = ourSide;
            switch (side){
                case RED:
                    TapeGoal = 20;
                    break;
                case BLUE:
                    TapeGoal = 25;
                    break;
                
            }
        }

        


  
    @Override
    public Mat processFrame(Mat inputMat) {
        min_deviation = 1000000;
        min_deviation2 = 1000000;
        min_deviation3 = 1000000;
        box2xfinal = 0;
        box2yfinal = 0;
        box3xfinal = 0;
        box3yfinal = 0;
        box1x = startx;
        box1y = starty;
        Mat RGBMat = inputMat;
        //Converts to HSV
        Imgproc.cvtColor(inputMat, inputMat, Imgproc.COLOR_RGB2HSV);
        while (box1y < maxy &! l.isStopRequested()) {

            while (box1x < maxx &! l.isStopRequested()){
         //Defines an anchor point for the first box using the dimensions
         Point BOX1_TOPLEFT_ANCHOR_POINT = new Point(box1x, box1y);
            
         //Defines the two points used to draw the first box
         Point box1_pointA = new Point(
             BOX1_TOPLEFT_ANCHOR_POINT.x,
             BOX1_TOPLEFT_ANCHOR_POINT.y);
         Point box1_pointB = new Point(
             BOX1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
             BOX1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            //Sets the box1 Mat to the stuff in box1
            box1_Mat = inputMat.submat(new Rect(box1_pointA, box1_pointB));

            //Takes the average hue of box1
            box1_average_hue = (int) Core.mean(box1_Mat).val[0];

            //Does the same for saturation
            box1_average_sat = (int) Core.mean(box1_Mat).val[1];

            box1_deviation = Math.abs(box1_average_hue - HueGoal);

            if (box1_deviation < min_deviation && box1_average_sat > sat_threshold){
                min_deviation = box1_deviation;
                box1xfinal = box1x;
                box1yfinal = box1y;
                finalsat = box1_average_sat;
                besthue = box1_average_hue;

            }
            box1x++;
            }
            box1x = startx;
            box1y++;

        }
         /* 
        box2x = startx;
        box2y = maxy - 1;
        box2xfinal = startx;
        box2yfinal = maxy - 1;

      while (box2y < maxy &! l.isStopRequested()) {

            while (box2x < maxx &! l.isStopRequested()){
         //Defines an anchor point for the first box using the dimensions
         Point BOX2_TOPLEFT_ANCHOR_POINT = new Point(box2x, box2y);
            
         //Defines the two points used to draw the first box
         Point box2_pointA = new Point(
             BOX2_TOPLEFT_ANCHOR_POINT.x,
             BOX2_TOPLEFT_ANCHOR_POINT.y);
         Point box2_pointB = new Point(
             BOX2_TOPLEFT_ANCHOR_POINT.x + TAPE_WIDTH,
             BOX2_TOPLEFT_ANCHOR_POINT.y + TAPE_HEIGHT);
            //Sets the box1 Mat to the stuff in box1
            box2_Mat = inputMat.submat(new Rect(box2_pointA, box2_pointB));

            //Takes the average hue of box1
            box2_average_hue = (int) Core.mean(box2_Mat).val[0];

            //Does the same for saturation
            box2_average_sat = (int) Core.mean(box2_Mat).val[1];

            box2_deviation = Math.abs(box2_average_hue - TapeGoal);

            //Includes a term to basically avoid accidentally detecting the cup
            if (box2_deviation + 2 < min_deviation2 && box2_average_sat > sat_threshold && (Math.abs(box2x - box1xfinal - REGION_WIDTH/2) > 40)){
                min_deviation2 = box2_deviation;
                box2xfinal = box2x;
                box2yfinal = box2y;

            }
            box2x++;
            }
            box2x = startx;
            box2y++;

        }

        box3x = box2xfinal + TAPE_WIDTH;
        box3y = maxy - 1;
        box3xfinal = box2xfinal + TAPE_WIDTH;
        box3yfinal = maxy - 1;

        while (box3y < maxy &! l.isStopRequested()) {

            while (box3x < maxx &! l.isStopRequested()){
         //Defines an anchor point for the first box using the dimensions
         Point BOX3_TOPLEFT_ANCHOR_POINT = new Point(box3x, box3y);
            
         //Defines the two points used to draw the first box
         Point box3_pointA = new Point(
             BOX3_TOPLEFT_ANCHOR_POINT.x,
             BOX3_TOPLEFT_ANCHOR_POINT.y);
         Point box3_pointB = new Point(
             BOX3_TOPLEFT_ANCHOR_POINT.x + TAPE_WIDTH,
             BOX3_TOPLEFT_ANCHOR_POINT.y + TAPE_HEIGHT);
            //Sets the box1 Mat to the stuff in box1
            box3_Mat = inputMat.submat(new Rect(box3_pointA, box3_pointB));

            //Takes the average hue of box1
            box3_average_hue = (int) Core.mean(box3_Mat).val[0];

            //Does the same for saturation
            box3_average_sat = (int) Core.mean(box3_Mat).val[1];

            box3_deviation = Math.abs(box3_average_hue - TapeGoal);

            //Includes a term to basically avoid accidentally detecting the cup
            if (box3_deviation + 5< min_deviation3 && box3_average_sat > sat_threshold && (Math.abs(box3x - box1xfinal - REGION_WIDTH/2) > 40)){
                min_deviation3 = box3_deviation;
                box3xfinal = box3x;
                box3yfinal = box3y;

            }
            box3x++;
            }
            box3x = box2xfinal + TAPE_WIDTH;
            box3y++;

        }
        
          //Defines an anchor point for the first box using the dimensions
         Point BOX4_TOPLEFT_ANCHOR_POINT = new Point(box2x, box2y);
            
         //Defines the two points used to draw the first box
         Point box4_pointA = new Point(
             BOX4_TOPLEFT_ANCHOR_POINT.x,
             BOX4_TOPLEFT_ANCHOR_POINT.y);
         Point box4_pointB = new Point(
             BOX4_TOPLEFT_ANCHOR_POINT.x + TAPE_WIDTH,
             BOX4_TOPLEFT_ANCHOR_POINT.y + TAPE_HEIGHT);
            //Sets the box1 Mat to the stuff in box1
            box4_Mat = RGBMat.submat(new Rect(box4_pointA, box4_pointB));

            //Takes the average hue of box1
            box4_average_hue = (int) Core.mean(box4_Mat).val[0];

            //Does the same for saturation
            box4_average_sat = (int) Core.mean(box4_Mat).val[1];
            
            //Does the same for saturation
            int box4_average_val = (int) Core.mean(box4_Mat).val[2];
            
            l.telemetry.addData("2nd", box4_average_sat);
            l.telemetry.addData("1st", box4_average_hue);
            l.telemetry.addData("3rd", box4_average_val);
            
            
*/

// If we are on the red side, the right will have a small area of detection so we default there. The opposite is true for the blue side.
if (side == Side.RED) {
    // Checks where the best fit was and gives analysis accordinglt
            if (box1xfinal > line2x || min_deviation > 30){
                position = Position.RIGHT;
            } else if (box1xfinal < line1x){
                position = Position.LEFT;
            } else {
                position = Position.MIDDLE;
            }
        } else if (box1xfinal < line1x || min_deviation > 30){
                position = Position.LEFT;
            } else if (box1xfinal > line2x){
                position = Position.RIGHT;
            } else {
                position = Position.MIDDLE;
            }


        //Defines an anchor point for the best box using the dimensions we recorded
        Point FINALBOX_TOPLEFT_ANCHOR_POINT = new Point(box1xfinal, box1yfinal);
      /*  Point TAPE1BOX_TOPLEFT_ANCHOR_POINT = new Point(box2xfinal, box2yfinal);
        Point TAPE2BOX_TOPLEFT_ANCHOR_POINT = new Point(box3xfinal, box3yfinal);
        Point TAPE3BOX_TOPLEFT_ANCHOR_POINT = new Point(box4x, box4y);
        */
            
        //Defines the two points used to draw the final box
        Point finalbox_pointA = new Point(
            FINALBOX_TOPLEFT_ANCHOR_POINT.x,
            FINALBOX_TOPLEFT_ANCHOR_POINT.y);
        Point finalbox_pointB = new Point(
            FINALBOX_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            FINALBOX_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
       /* Point tape1box_pointA = new Point(
                TAPE1BOX_TOPLEFT_ANCHOR_POINT.x,
                TAPE1BOX_TOPLEFT_ANCHOR_POINT.y);
        Point tape1box_pointB = new Point(
                TAPE1BOX_TOPLEFT_ANCHOR_POINT.x + TAPE_WIDTH,
                TAPE1BOX_TOPLEFT_ANCHOR_POINT.y + TAPE_HEIGHT);
        Point tape2box_pointA = new Point(
                TAPE2BOX_TOPLEFT_ANCHOR_POINT.x,
                TAPE2BOX_TOPLEFT_ANCHOR_POINT.y);
        Point tape2box_pointB = new Point(
                TAPE2BOX_TOPLEFT_ANCHOR_POINT.x + TAPE_WIDTH,
                TAPE2BOX_TOPLEFT_ANCHOR_POINT.y + TAPE_HEIGHT);
        Point tape3box_pointA = new Point(
                TAPE3BOX_TOPLEFT_ANCHOR_POINT.x,
                TAPE3BOX_TOPLEFT_ANCHOR_POINT.y);
        Point tape3box_pointB = new Point(
                TAPE3BOX_TOPLEFT_ANCHOR_POINT.x + TAPE_WIDTH,
                TAPE3BOX_TOPLEFT_ANCHOR_POINT.y + TAPE_HEIGHT);
                */
            
            //Defines a bunch of points that we use to draw lines that will show where we are looking
            Point line0pointA = new Point(startx, starty);
            Point line0pointB = new Point(startx, maxy + REGION_HEIGHT);
            Point line1pointA = new Point(line1x +REGION_WIDTH/2, starty); //Splits the difference with region width - where the box falls more is where we detect
            Point line1pointB = new Point(line1x +REGION_WIDTH/2, maxy + REGION_HEIGHT);
            Point line2pointA = new Point(line2x +REGION_WIDTH/2, starty);
            Point line2pointB = new Point(line2x +REGION_WIDTH/2, maxy + REGION_HEIGHT);
            Point line3pointA = new Point(startx, starty);
            Point line3pointB = new Point(maxx + REGION_WIDTH, starty);
            Point line4pointA = new Point(maxx + REGION_WIDTH, starty);
            Point line4pointB = new Point(maxx + REGION_WIDTH, maxy + REGION_HEIGHT); //Shows right end of box
            Point line5pointA = new Point(startx, maxy + REGION_HEIGHT);
            Point line5pointB = new Point(maxx + REGION_WIDTH, maxy + REGION_HEIGHT); //Shows bottom end of box
            
             
            //Draws the best box
            Imgproc.rectangle(
            inputMat, // What to draw on
            finalbox_pointA, // First point which defines the rectangle
            finalbox_pointB, // Second point which defines the rectangle
            RED, // The color the rectangle is drawn in
            2); // Thickness of the rectangle lines
            /*
            Imgproc.rectangle(
            inputMat, // What to draw on
            tape1box_pointA, // First point which defines the rectangle
            tape1box_pointB, // Second point which defines the rectangle
            BLACK, // The color the rectangle is drawn in
            2); // Thickness of the rectangle lines
            Imgproc.rectangle(
            inputMat, // What to draw on
            tape2box_pointA, // First point which defines the rectangle
            tape2box_pointB, // Second point which defines the rectangle
            WHITE, // The color the rectangle is drawn in
            2); // Thickness of the rectangle lines
            
            Imgproc.rectangle(
            inputMat, // What to draw on
            tape3box_pointA, // First point which defines the rectangle
            tape3box_pointB, // Second point which defines the rectangle
            RED, // The color the rectangle is drawn in
            2); // Thickness of the rectangle lines
                
                */
            //Draws all the boundary lines
            Imgproc.line(
            inputMat, // What to draw on
            line0pointA, // First point which defines the rectangle
            line0pointB, // Second point which defines the rectangle
            RED, // The color the rectangle is drawn in
            2); // Thickness of the rectangle lines
          
            Imgproc.line(
            inputMat, // What to draw on
            line1pointA, // First point which defines the rectangle
            line1pointB, // Second point which defines the rectangle
            RED, // The color the rectangle is drawn in
            2); // Thickness of the rectangle lines
            
            Imgproc.line(
            inputMat, // What to draw on
            line2pointA, // First point which defines the rectangle
            line2pointB, // Second point which defines the rectangle
            RED, // The color the rectangle is drawn in
            2); // Thickness of the rectangle lines
            
            Imgproc.line(
            inputMat, // What to draw on
            line3pointA, // First point which defines the rectangle
            line3pointB, // Second point which defines the rectangle
            RED, // The color the rectangle is drawn in
            2); // Thickness of the rectangle lines
            
            Imgproc.line(
            inputMat, // What to draw on
            line4pointA, // First point which defines the rectangle
            line4pointB, // Second point which defines the rectangle
            RED, // The color the rectangle is drawn in
            2); // Thickness of the rectangle lines
            
            Imgproc.line(
            inputMat, // What to draw on
            line5pointA, // First point which defines the rectangle
            line5pointB, // Second point which defines the rectangle
            RED, // The color the rectangle is drawn in
            2); // Thickness of the rectangle lines
            
        l.telemetry.addData("Box x", box1xfinal);
        l.telemetry.addData("Box y", box1yfinal);
        l.telemetry.addData("Deviation", min_deviation);
        l.telemetry.addData("Saturation", finalsat);
        l.telemetry.addData("Hue", besthue);
        l.telemetry.addData("tape1Dev", min_deviation2);
        l.telemetry.addData("tape2Dev", min_deviation3);
        l.telemetry.update();
        //Shows the image
        return inputMat;
        
    }
    
}