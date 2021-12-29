/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class EasyOpenCVExample extends LinearOpMode
{
    //OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;
    OpenCvCamera webcam;

    @Override
    public void runOpMode()
    {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);


        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);  doesn't work with webcam

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 10);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.addData("HERE", "");
        telemetry.update();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Analysis Right", pipeline.avg1GetAnalysis());
            telemetry.addData("Analysis Middle", pipeline.avg2GetAnalysis());
            telemetry.addData("Analysis Left", pipeline.avg3GetAnalysis());
            telemetry.addData("Analysis","Top Left: %d Top Right: %d", pipeline.getTopLeftAvg(),pipeline.getTopRightAvg());



            telemetry.addData("Position", pipeline.position);
            telemetry.update();
            dashboardTelemetry.addData("Analysis Right", pipeline.avg1GetAnalysis());
            dashboardTelemetry.addData("Analysis Middle", pipeline.avg2GetAnalysis());
            dashboardTelemetry.addData("Analysis Left", pipeline.avg3GetAnalysis());
            dashboardTelemetry.addData("Analysis","Top Left: %d Top Right: %d", pipeline.getTopLeftAvg(),pipeline.getTopRightAvg());
            dashboardTelemetry.addData("robot pos", pipeline.robotPosition);
            dashboardTelemetry.addData("Position", pipeline.position);
            dashboardTelemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    @Config
    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        public static int regionWidth = 60;
        public static int regionHeight = 60;

        /*
         * An enum to define the ring position
         */
        public enum duckPosition {
            LEFT,
            MIDDLE,
            RIGHT,
            NONE,
        }

        public  enum robotPos{
            LEFT,
            RIGHT,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(245,150);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(170,150);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(80,150);
        static final Point LEFT_TOPLEFT_ANCHOR_POINT = new Point(0,50);
        static final Point RIGHT_TOPLEFT_ANCHOR_POINT = new Point(250,50);


        static int REGION_WIDTH = regionWidth;
        static int REGION_HEIGHT = regionHeight;

        final int DUCK_THRESHOLD = 40;

        Point right_region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point right_region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + regionWidth,
                REGION1_TOPLEFT_ANCHOR_POINT.y + regionHeight);

        Point middle_region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point middle_region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + regionWidth,
                REGION2_TOPLEFT_ANCHOR_POINT.y+regionHeight);

        Point left_region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point left_region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + regionWidth,
                REGION3_TOPLEFT_ANCHOR_POINT.y+regionHeight);

        Point topLeftRegionA = new Point(
                LEFT_TOPLEFT_ANCHOR_POINT.x,
                LEFT_TOPLEFT_ANCHOR_POINT.y);
        Point topLeftRegionB = new Point(
                LEFT_TOPLEFT_ANCHOR_POINT.x + regionWidth,
                LEFT_TOPLEFT_ANCHOR_POINT.y + regionHeight);

        Point topRightRegionA = new Point(
                RIGHT_TOPLEFT_ANCHOR_POINT.x,
                RIGHT_TOPLEFT_ANCHOR_POINT.y);
        Point topRightRegionB = new Point(
                RIGHT_TOPLEFT_ANCHOR_POINT.x + regionWidth,
                RIGHT_TOPLEFT_ANCHOR_POINT.y + regionHeight);





        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat region2_Cb;
        Mat region3_Cb;

        Mat topLeft_Cb;
        Mat topRight_Cb;

        Mat YCrCb = new Mat();
        Mat Cr = new Mat();
        Mat Cr2 = new Mat();
        Mat Cr3 = new Mat();

        int avg1;
        int avg2;
        int avg3;

        int topLeftAvg;
        int topRightAvg;

        Mat hsv = new Mat();
        Scalar lowHSV1 = new Scalar(0, 100, 20); // lower bound HSV for red
        Scalar highHSV1 = new Scalar(10,255,255); // higher bound HSV for red

        Scalar lowHSV2 = new Scalar(160, 100, 20); // lower bound HSV for red
        Scalar highHSV2 = new Scalar(179,255,255); // higher bound HSV for red
        Scalar blueLowHSV = new Scalar(100,150,0);
        Scalar blueHighHSV = new Scalar(140,255,255);





        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile duckPosition position = duckPosition.NONE;
        private volatile robotPos robotPosition = robotPos.NONE;



        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable

        /*void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 0);
            Imgproc.cvtColor(input,hsv, Imgproc.COLOR_RGB2HSV);
        }*/
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(hsv, Cr, 0);
            Core.inRange(hsv,lowHSV1,highHSV1,Cr);
            Core.inRange(hsv,lowHSV2,highHSV2,Cr);
        }



        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);
            region1_Cb = Cr.submat(new Rect(right_region1_pointA, right_region1_pointB));
            region2_Cb = Cr.submat(new Rect(middle_region2_pointA, middle_region2_pointB));
            region3_Cb = Cr.submat(new Rect(left_region3_pointA,left_region3_pointB));
            topLeft_Cb = Cr.submat(new Rect(topLeftRegionA,topLeftRegionB));
            topRight_Cb = Cr.submat(new Rect(topRightRegionA,topLeftRegionB));



        }

        @Override
        public Mat processFrame(Mat input)
        {
            right_region1_pointB.x = REGION1_TOPLEFT_ANCHOR_POINT.x + regionWidth;
            right_region1_pointB.y = REGION1_TOPLEFT_ANCHOR_POINT.y + regionHeight;

            middle_region2_pointB.x = REGION2_TOPLEFT_ANCHOR_POINT.x + regionWidth;
            middle_region2_pointB.y = REGION2_TOPLEFT_ANCHOR_POINT.y + regionHeight;

            left_region3_pointB.x = REGION3_TOPLEFT_ANCHOR_POINT.x + regionWidth;
            left_region3_pointB.y = REGION3_TOPLEFT_ANCHOR_POINT.y + regionHeight;


            inputToCb(input);
            Mat mat = new Mat();

            Mat thresh = new Mat();

            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            topLeftAvg = (int) Core.mean(topLeft_Cb).val[0];

            topRightAvg = (int) Core.mean(topRight_Cb).val[0];




            Imgproc.rectangle(
                    input, // Buffer to draw on
                    right_region1_pointA, // First point which defines the rectangle
                    right_region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    middle_region2_pointA, // First point which defines the rectangle
                    middle_region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    left_region3_pointA, // First point which defines the rectangle
                    left_region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    topLeftRegionA, // First point which defines the rectangle
                    topLeftRegionB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    topRightRegionA, // First point which defines the rectangle
                    topRightRegionB,// Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill



            position = duckPosition.NONE; // Record our analysis
            if(avg1 > avg2 && avg1 > avg3){
                position = duckPosition.RIGHT;
            }else if (avg2 > avg1 && avg2 > avg3){
                position = duckPosition.MIDDLE;
            }else if(avg3 > avg1 && avg3 > avg2){
                position = duckPosition.LEFT;
            }

            robotPosition = robotPos.NONE;
            if(topLeftAvg > topRightAvg){
                robotPosition = robotPos.RIGHT;
            }else if(topRightAvg > topLeftAvg){
                robotPosition = robotPos.LEFT;
            }

            return input;
        }

        public int avg1GetAnalysis()  {  return avg1;             }
        public int avg2GetAnalysis()  {  return avg2;             }
        public int avg3GetAnalysis()  {  return avg3;             }
        public int getTopLeftAvg()    {  return topLeftAvg;       }
        public int getTopRightAvg()   {  return topRightAvg;      }




    }
}