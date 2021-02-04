package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.*;
import org.openftc.easyopencv.*;
import org.opencv.imgproc.Imgproc;

@TeleOp(name="ring detector")
public class RingDetector extends OpMode {
    // 0 1 4
    // auto goals:
    // deliver wobble goal to correct depot corresponding with ring amt: 15
    // park over launch line: 5
    // low goal: 3, mid goal: 6, high goal: 12, power shot: 15
    // must be within launch zone to launch into mid goal, high goal, and power shot


    OpenCvCamera phoneCam;
    RingDetectorPipeline pipeline;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new RingDetectorPipeline(true, 30);
        phoneCam.setPipeline(pipeline);

        // fixes the problems we had last year with the camera preview on the phone not being in landscape or whatever
        // also starts the camera
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

    }

    @Override
    public void loop() {
        telemetry.addLine("Rings: " + pipeline.getRingAmount());
        telemetry.addLine(pipeline.getTelemetry());
    }



    private static class RingDetectorPipeline extends OpenCvPipeline {
        int colorComponentNum;
        int threshold;
        public RingDetectorPipeline(boolean isCr, int threshold) {
            colorComponentNum = isCr ? 1 : 2;
            this.threshold = threshold;
        }

        private enum RingAmount {
            NONE,
            ONE,
            FOUR,
        }

        // telemetry
        double average1;
        double average2;
        double difference1;
        double difference2;

        // color constants used for rectangles drawn on the phone like last yr
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        static final int baseAverage = 128;

        // constants for defining the area of rectangles
        static final Point FOUR_RING_TOP_LEFT_ANCHOR = new Point(100, 75);
        static final Point ONE_RING_TOP_LEFT_ANCHOR = new Point(100, 125);
        static final int REC_WIDTH = 100;
        static final int REC_HEIGHT = 50;
        static final int ONE_RING_WIDTH = 100;
        static final int ONE_RING_HEIGHT = 30;

        // using the constants we calculate the points actually used for the rectangles
        // point a would be the top left point, point b would be the bottom right (creating a diagonal)

        Point four_ring_pointA = FOUR_RING_TOP_LEFT_ANCHOR;
        Point four_ring_pointB = new Point(FOUR_RING_TOP_LEFT_ANCHOR.x + REC_WIDTH, FOUR_RING_TOP_LEFT_ANCHOR.y + REC_HEIGHT);
        Point one_ring_pointA = ONE_RING_TOP_LEFT_ANCHOR;
        Point one_ring_pointB = new Point(ONE_RING_TOP_LEFT_ANCHOR.x + ONE_RING_WIDTH, ONE_RING_TOP_LEFT_ANCHOR.y + ONE_RING_HEIGHT);

        // computation vars
        // region1 is bound by the first the points
        // region2 is bound by the second 2
        Mat region1_Cr, region2_Cr;
        Mat YCrCb = new Mat();
        Mat component = new Mat();
        int avg1, avg2;

        private volatile RingAmount ringAmt;

        // turns rgb input into YCrCb
        void rgbToComponent(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, component, colorComponentNum);
        }

        @Override
        public void init(Mat firstFrame) {
            rgbToComponent(firstFrame);

            region1_Cr = component.submat(new Rect(four_ring_pointA, four_ring_pointB));
            region2_Cr = component.submat(new Rect(one_ring_pointA, one_ring_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            /*
             * - Explanation of whats happening here - (I'm basically a god commenter at this point amiright)
             * We convert the colors from RGB to YCrCb
             * This is because the vision code can better view the
             * region without being affected by darkness/brightness since that
             * will be in the Y channel instead of the Cb or Cr channel.
             * In rgb this would've been reflected over all 3 channels, R, G, and B
             * This link was really good at explaining how YCrCb compares to other color spaces
             * It also shows the differences in OpenCV code
             * https://www.learnopencv.com/color-spaces-in-opencv-cpp-python/
             *
             * We only need the Cr channel from the color-space since that reflects
             * the warm color the best
             * */

            // changes color space
            rgbToComponent(input);

            // get avg of each region
            avg1 = (int) Core.mean(region1_Cr).val[0];
            avg2 = (int) Core.mean(region2_Cr).val[0];
            average1 = avg1;
            average2 = avg2;

            /*
             * draws rectangles on the phone like last year's vision code
             * honestly the most retarded part of the code
             * it took me so long to manually estimate where the rectangles should be
             * until i realized that i could simply make rectangles based on the initial points that i made
             * yeah.. kinda stupid of me
             *
             * since the rectangles will be overlapped i thought its better to make them a separate color/thickness
             */

            // big region for 4 rings
            Imgproc.rectangle(
                    input, // what to draw on
                    four_ring_pointA, // top left pt
                    four_ring_pointB, // bottom right pt
                    BLUE, // color
                    1
            );

            Imgproc.rectangle(
                    input,
                    one_ring_pointA,
                    one_ring_pointB,
                    GREEN,
                    1 // i guess just make it 1 since its inside of the other region? idk
            );


            int diff1 = Math.abs(avg1 - baseAverage);
            int diff2 = Math.abs(avg2 - baseAverage);
            difference1 = diff1;
            difference2 = diff2;
            if(diff1 < threshold && diff2 < threshold) {
                ringAmt = RingAmount.NONE;
            } else if(diff1 > threshold) {
                ringAmt = RingAmount.FOUR;
            } else {
                ringAmt = RingAmount.ONE;
            }
            return input;
        }

        public RingAmount getRingAmount() {
            return ringAmt;
        }

        public String getTelemetry() {
            String newLine = System.getProperty("line.separator");
            return "avg1: " + average1 + " avg2: " + average2  + newLine + " diff1: " + difference1 + " diff2: " + difference2;
        }
    }
}