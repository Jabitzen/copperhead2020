package org.firstinspires.ftc.teamcode;


import android.graphics.Bitmap;

import static android.graphics.Color.red;
import static android.graphics.Color.green;
import static android.graphics.Color.blue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters;

import java.util.ArrayList;
import java.util.concurrent.BlockingQueue;

@TeleOp(name="bitMapTests", group="Pushbot")
public class bitMapTests extends LinearOpMode {

    private VuforiaLocalizer vuforia;
    private Parameters parameters;
    private CameraDirection CAMERA_CHOICE = CameraDirection.BACK;

    private final int RED_THRESHOLD = 35;
    private final int GREEN_THRESHOLD = 35;
    private final int BLUE_THRESHOLD = 35;

    private static final String VUFORIA_KEY = "Aad5SFz/////AAABmT+V+odOO0Lcr/j0iQ+cxkFwNC+TZ2mUCftLAt5wXY/BCUwwq4iU84o/Q15qndcAU2JPL+SN/qG+8GE9j0fDBHGkUHqAOBhu41XjysYMyF+kBeicTqdwfzaUzT5NLSvAU8aOZ+oIyQu+KFBoHVGylT7Yf6ASJw9gX34kck2ECReLzij/3gORcXvFFtXm/rGIyQGlxPGOXDv0ZnYmurQ79fH6fnArDP6Ylcc9QYQuPInhQ7BKzhiicPZhciwnKfZfa3CH4zu8zSWajDLDQmaj3iZxnkVsXac6XCk2cYMD37svmgGqI78M4bAcRUfOpPqGtqz+89jC8kUmCip8OSdmGO7ArKzBPHF3sctfMxurkuG4";

    private BlockingQueue<VuforiaLocalizer.CloseableFrame> frame;

    public static String bitmapSkyStonePosition;
    //WebcamName webcamName = null;

    @Override
    public void runOpMode() throws InterruptedException {

        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = this.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        params.vuforiaLicenseKey = VUFORIA_KEY;
        //params.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(params);


        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(4); //tells VuforiaLocalizer to only store one frame at a time
        vuforia.enableConvertFrameToBitmap();

        waitForStart();


        //getBitmap();
        colors();
        //sample();

        //telemetry.addData("width", bm)


    }



    private Bitmap getBitmap2() throws InterruptedException{
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        Image rgb = null;

        long numImages = frame.getNumImages();

        for (int i = 0; i < numImages; i++) {
            Image img = frame.getImage(i);

            int fmt = img.getFormat();

            if (fmt == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;

            }

        }
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        frame.close();

        telemetry.addLine("Got Bitmap");
        telemetry.update();

        return bm;

    }

    public Bitmap getBitmap() throws InterruptedException{

        VuforiaLocalizer.CloseableFrame picture;
        picture = vuforia.getFrameQueue().take();
        Image rgb = picture.getImage(1);

        long numImages = picture.getNumImages();

        telemetry.addData("Num Images", numImages);
        telemetry.update();

        for (int i = 0; i < numImages; i++) {
            int format = picture.getImage(i).getFormat();
            if (format == PIXEL_FORMAT.RGB565) {
                rgb = picture.getImage(i);
                break;

            }

            else {
                telemetry.addLine("Didn't find correct RGB format");
                telemetry.update();


            }

        }

        Bitmap imageBitmap = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        imageBitmap.copyPixelsFromBuffer(rgb.getPixels());

        telemetry.addData("Image width", imageBitmap.getWidth());
        telemetry.addData("Image height", imageBitmap.getHeight());
        telemetry.update();




        picture.close();

        telemetry.addLine("Got bitmap");
        telemetry.update();

        return imageBitmap;
    }

    public String sample() throws InterruptedException{
        Bitmap bitmap = getBitmap();
        String bitmapCubePosition;



        ArrayList<Integer> xValues = new ArrayList<>();

        int avgX = 0;

        //top left = (0,0)
        int colNum = 193;
        int end = 233;

        sleep(2000);
        while (end <= 1280) {
            while (colNum < end) {
                for (int rowNum = 0; rowNum < (int) (bitmap.getHeight()); rowNum ++) {
                    int pixel = bitmap.getPixel(colNum, rowNum);

                    int redPixel = red(pixel);
                    int greenPixel = green(pixel);
                    int bluePixel = blue(pixel);

                    if (redPixel <= RED_THRESHOLD && greenPixel <= GREEN_THRESHOLD && bluePixel <= BLUE_THRESHOLD) {
                        xValues.add(colNum);

                    }
                }
                colNum ++;

            }
           colNum += 387;
            end += 427;
        }
//        telemetry.addLine("2");
//        sleep(2000);
//        for (int colNum = 0; colNum < bitmap.getWidth(); colNum ) {
//
//            for (int rowNum = 0; rowNum < (int)(bitmap.getHeight() ); rowNum += 3) {
//                int pixel = bitmap.getPixel(colNum, rowNum);
//
//                int redPixel = red(pixel);
//                int greenPixel = green(pixel);
//                int bluePixel = blue(pixel);
//
//                if (redPixel <= RED_THRESHOLD && greenPixel <= GREEN_THRESHOLD && bluePixel <= BLUE_THRESHOLD) {
//                    xValues.add(colNum);
//
//                }
//
//            }
//
//        }

        for (int x : xValues) {
            avgX+= x;
        }

        avgX /= xValues.size();

        if (avgX<= 650) {
            bitmapCubePosition = "left";

        }
        else if (650 < avgX && avgX <= 800) {
            bitmapCubePosition = "center";

        }

        else {
            bitmapCubePosition = "right";
        }

        telemetry.addData("width", bitmap.getWidth());
        telemetry.update();
        sleep(2000);

        telemetry.addLine("test 3");
        sleep(2000);


        telemetry.addData("Cube Position", bitmapCubePosition);
        telemetry.addData ("X-value", avgX);

        telemetry.update();
        sleep (10000);
        return bitmapCubePosition;

    }

    public void colors() throws InterruptedException{
        Bitmap bitmap = getBitmap();
        String bitmapCubePosition;

        ArrayList<Double> blueValues = new ArrayList<>();
        ArrayList<Double> redValues = new ArrayList<>();
        ArrayList<Double> greenValues = new ArrayList<>();
        ArrayList<Double> xValues = new ArrayList<>();

        double x1 = 0;
        double x2 = 0;
        double x3 = 0;

        double avg1 = 0;
        double avg2 = 0;
        double avg3 = 0;

        double green1 = 0;
        double green2 = 0;
        double green3 = 0;

        double red1 = 0;
        double red2 = 0;
        double red3 = 0;


      /*  // for looking at 3 stones with phone cam
        int colNum =203;
        int end = 223;
        telemetry.addData("width", bitmap.getWidth());
        telemetry.update();

        sleep(2000);
        int count = 0;
        while (count < 3) {
            while (colNum < end) {
                for (int rowNum = 0; rowNum < (720); rowNum ++) {
                    int pixel = bitmap.getPixel(colNum, rowNum);
                    blueValues.add((double)blue(pixel));
                    redValues.add((double)red(pixel));
                    greenValues.add((double)green(pixel));
                }
                colNum ++;
            }
            colNum += 430;
            end += 450;
            count ++;
        } */

      //for looking at 2 skystones with phone cam
        int colNum =203;
        int end = 223;
        telemetry.addData("width", bitmap.getWidth());
        telemetry.update();

        sleep(2000);
        int count = 0;
        while (count < 3) {
            while (colNum < end) {
                for (int rowNum = 0; rowNum < (720); rowNum ++) {
                    int pixel = bitmap.getPixel(colNum, rowNum);
                    blueValues.add((double)blue(pixel));
                    redValues.add((double)red(pixel));
                    greenValues.add((double)green(pixel));
                }
                colNum ++;
            }
            colNum += 430;
            end += 450;
            count ++;
        }



//        for (int colNum = 0; colNum < bitmap.getWidth(); colNum ) {
//
//            for (int rowNum = 0; rowNum < (int)(bitmap.getHeight() ); rowNum += 3) {
//                int pixel = bitmap.getPixel(colNum, rowNum);
//
//                int redPixel = red(pixel);
//                int greenPixel = green(pixel);
//                int bluePixel = blue(pixel);
//
//                if (redPixel <= RED_THRESHOLD && greenPixel <= GREEN_THRESHOLD && bluePixel <= BLUE_THRESHOLD) {
//                    xValues.add(colNum);
//
//                }
//
//            }
//
//        }
        double size = blueValues.size();
        telemetry.addData("blue size", size );
        telemetry.update();
        sleep(5000);

/*
        for (int i = 0; i < size; i++) {
            if (i <= size / 3.0) {
                avg1 += blueValues.get(i);

            }

            else if (i > size / 3.0 && i <= size * .666) {
                avg2 += blueValues.get(i);
            }

            else {
                avg3 += blueValues.get(i);
            }
        }
*/

        double index = 0;
        double stop = size / 3.0;

        while (index < stop){
            avg1 += blueValues.get((int)index);
            index += 1.0;
        }

        index = size / 3.0;
        stop = size * (.66666);

        while (index < stop){
            avg2 += blueValues.get((int)index);
            index += 1.0;
        }

        index = size * 0.66666;
        stop = size;

        while (index < stop){
            avg3 += blueValues.get((int)index);
            index += 1.0;
        }


        if (red1 < 1800000) {
            bitmapCubePosition = "left";
        }

        else if (red2 < 1800000) {
            bitmapCubePosition = "center";
        }

        else if (red3 < 1800000) {
            bitmapCubePosition = "right";
        }

        else {
            bitmapCubePosition = "oopsie";
        }

        telemetry.addData("Cube Position", bitmapCubePosition);








        double size2 = redValues.size();

        telemetry.addData("red size", size2 );
        telemetry.update();
        sleep(5000);

        for (int i = 0; i < size2; i++) {
            if (i <= size2 / 3) {
                red1 += redValues.get(i);

            }

            else if (i > size2 / 3 && i <= size2 * 2/3) {
                red2 += redValues.get(i);
            }

            else {
                red3 += redValues.get(i);
            }
        }





        double size3 = greenValues.size();


        telemetry.addData("green size", size3 );
        telemetry.update();
        sleep(5000);

        for (int i = 0; i < size3; i++) {
            if (i <= size3 / 3) {
                green1 += greenValues.get(i);

            }

            else if (i > size3 / 3 && i <= size3 * 2/3) {
                green2 += greenValues.get(i);
            }

            else {
                green3 += greenValues.get(i);
            }
        }



        //telemetry.addData("size of bitmap", blueValues.size());


        telemetry.addData("avg1", avg1);
        telemetry.addData ("avg2", avg2);
        telemetry.addData ("avg3", avg3);

        telemetry.update();
        sleep (8000);


        telemetry.addData("green1", green1);
        telemetry.addData ("green2", green2);
        telemetry.addData ("green3", green3);

        telemetry.update();
        sleep (8000);

        telemetry.addData("red1", red1);
        telemetry.addData ("red2", red2);
        telemetry.addData ("red3", red3);

        telemetry.update();
        sleep (8000);

    }

    public Bitmap vufConvertToBitmap(Frame frame) { return vuforia.convertFrameToBitmap(frame); }


}

