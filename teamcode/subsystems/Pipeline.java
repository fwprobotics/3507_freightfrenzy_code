// THIS IS OLD - DO NOT USE UNLESS LOOPY STOPS WORKING
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

public class Pipeline extends OpenCvPipeline {

    //Defines possible position outcomes
    public enum Position {
        LEFT,
        MIDDLE,
        RIGHT
    }

    //Defines dimensions for the boxes
    final int REGION_WIDTH = 25;
    final int REGION_HEIGHT = 30;




    //Defines starting points for the first box
    public static int box1x;
    public static int box1y;

    public static int box2x;
    public static int box2y;

    public static int box3x;
    public static int box3y;
    //
    public Pipeline(int x1, int y1, int x_increment, int y_increment) {
        box1x = x1;
        box1y = y1;
        box2x = (x1 + x_increment);
        box2y = (y1 + y_increment);
        box3x = (x1 + 2 * x_increment);
        box3y = (y1 + 2 * y_increment);
    }

    //Defines a Mat specifically for the first box
    Mat box1_Hue;

    //Defines an int to take the hue of the first box
    public int box1_average_hue;



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


    Mat box2_Hue;

    public int box2_average_hue;

    final Point BOX2_TOPLEFT_ANCHOR_POINT = new Point(box2x, box2y);

    Point box2_pointA = new Point(
        BOX2_TOPLEFT_ANCHOR_POINT.x,
        BOX2_TOPLEFT_ANCHOR_POINT.y);
    Point box2_pointB = new Point(
        BOX2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
        BOX2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // And the third

    Mat box3_Hue;

    public int box3_average_hue;

    final Point BOX3_TOPLEFT_ANCHOR_POINT = new Point(box3x, box3y);

    Point box3_pointA = new Point(
        BOX3_TOPLEFT_ANCHOR_POINT.x,
        BOX3_TOPLEFT_ANCHOR_POINT.y);
    Point box3_pointB = new Point(
        BOX3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
        BOX3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Sets ideal hue
    int HueGoal = 150;

    // ints to hold distance from hue goal
    public int box1_deviation;
    public int box2_deviation;
    public int box3_deviation;

    //Defines colors to draw boxes with
    final Scalar RED = new Scalar(225, 0, 0);

    final Scalar WHITE = new Scalar(225, 225, 225);

    final Scalar BLACK = new Scalar(0, 0, 0);

    // Sets up a variable to store our analysis and sets it to a default (what this is doesn't matter)
    public volatile Position position = Position.MIDDLE;


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
        if (box1_deviation < box2_deviation && box1_deviation < box3_deviation) {
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
