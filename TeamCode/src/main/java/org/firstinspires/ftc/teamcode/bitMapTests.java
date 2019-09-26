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

    private final int RED_THRESHOLD = 20;
    private final int GREEN_THRESHOLD = 20;
    private final int BLUE_THRESHOLD = 20;

    private static final String VUFORIA_KEY = "Aad5SFz/////AAABmT+V+odOO0Lcr/j0iQ+cxkFwNC+TZ2mUCftLAt5wXY/BCUwwq4iU84o/Q15qndcAU2JPL+SN/qG+8GE9j0fDBHGkUHqAOBhu41XjysYMyF+kBeicTqdwfzaUzT5NLSvAU8aOZ+oIyQu+KFBoHVGylT7Yf6ASJw9gX34kck2ECReLzij/3gORcXvFFtXm/rGIyQGlxPGOXDv0ZnYmurQ79fH6fnArDP6Ylcc9QYQuPInhQ7BKzhiicPZhciwnKfZfa3CH4zu8zSWajDLDQmaj3iZxnkVsXac6XCk2cYMD37svmgGqI78M4bAcRUfOpPqGtqz+89jC8kUmCip8OSdmGO7ArKzBPHF3sctfMxurkuG4";

    private BlockingQueue<VuforiaLocalizer.CloseableFrame> frame;

    public static String bitmapSkyStonePosition;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = this.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        params.vuforiaLicenseKey = VUFORIA_KEY;
        params.cameraDirection = CAMERA_CHOICE;
        vuforia = ClassFactory.getInstance().createVuforia(params);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(4); //tells VuforiaLocalizer to only store one frame at a time
        vuforia.enableConvertFrameToBitmap();

        waitForStart();


        //getBitmap();
        sample();

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
        int colNum = 180;
        int end = 260;
        telemetry.addLine("test 1");
        sleep(2000);
        while (end < 1280) {
            while (colNum < end) {
                for (int rowNum = 0; rowNum < (int) (bitmap.getHeight()); rowNum += 3) {
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
            colNum += 320;
            end += 400;
        }
        telemetry.addLine("2");
        sleep(2000);
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

        if (avgX < (1280 / 3.0)) {
            bitmapCubePosition = "left";

        }
        else if (avgX > (1280 / 3.0) && avgX < (bitmap.getWidth() * 2.0/3)) {
            bitmapCubePosition = "center";

        }
        else {
            bitmapCubePosition = "right";

        }
        telemetry.addLine("test 3");
        sleep(2000);


        telemetry.addData("Cube Position", bitmapCubePosition);
        telemetry.addData ("X-value", avgX);
        telemetry.update();
        sleep (10000);
        return bitmapCubePosition;

    }

    public Bitmap vufConvertToBitmap(Frame frame) { return vuforia.convertFrameToBitmap(frame); }


}

