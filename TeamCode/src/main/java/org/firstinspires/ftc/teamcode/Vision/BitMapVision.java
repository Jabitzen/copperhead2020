package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;
import android.graphics.ImageDecoder;
import android.graphics.drawable.Drawable;

import static android.graphics.Color.red;
import static android.graphics.Color.green;
import static android.graphics.Color.blue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.concurrent.BlockingQueue;


public class BitMapVision {

    private LinearOpMode opMode;
    private VuforiaLocalizer vuforia;
    private Parameters parameters;
    private CameraDirection CAMERA_CHOICE = CameraDirection.BACK;

    private final int RED_THRESHOLD = 20;
    private final int GREEN_THRESHOLD = 20;
    private final int BLUE_THRESHOLD = 20;

    private static final String VUFORIA_KEY = "Aad5SFz/////AAABmT+V+odOO0Lcr/j0iQ+cxkFwNC+TZ2mUCftLAt5wXY/BCUwwq4iU84o/Q15qndcAU2JPL+SN/qG+8GE9j0fDBHGkUHqAOBhu41XjysYMyF+kBeicTqdwfzaUzT5NLSvAU8aOZ+oIyQu+KFBoHVGylT7Yf6ASJw9gX34kck2ECReLzij/3gORcXvFFtXm/rGIyQGlxPGOXDv0ZnYmurQ79fH6fnArDP6Ylcc9QYQuPInhQ7BKzhiicPZhciwnKfZfa3CH4zu8zSWajDLDQmaj3iZxnkVsXac6XCk2cYMD37svmgGqI78M4bAcRUfOpPqGtqz+89jC8kUmCip8OSdmGO7ArKzBPHF3sctfMxurkuG4";

    private BlockingQueue<VuforiaLocalizer.CloseableFrame> frame;

    public static String bitmapSkyStonePosition;

    public BitMapVision(LinearOpMode opMode) {

        this.opMode = opMode;

        int cameraMonitorViewId = this.opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        params.vuforiaLicenseKey = VUFORIA_KEY;
        params.cameraDirection = CAMERA_CHOICE;
        vuforia = ClassFactory.getInstance().createVuforia(params);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(4); //tells VuforiaLocalizer to only store one frame at a time
        vuforia.enableConvertFrameToBitmap();

    }

    private Bitmap getBitmap2() throws InterruptedException, IOException {

        //File file = new File(...);
        //ImageDecoder.Source source = ImageDecoder.createSource(file);
        //Drawable drawable = ImageDecoder.decodeDrawable(source);
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

        opMode.telemetry.addLine("Got Bitmap");
        opMode.telemetry.update();

        return bm;

    }

    public Bitmap getBitmap() throws InterruptedException{

        VuforiaLocalizer.CloseableFrame picture;
        picture = vuforia.getFrameQueue().take();
        Image rgb = picture.getImage(1);

        long numImages = picture.getNumImages();

        opMode.telemetry.addData("Num Images", numImages);
        opMode.telemetry.update();

        for (int i = 0; i < numImages; i++) {
            int format = picture.getImage(i).getFormat();
            if (format == PIXEL_FORMAT.RGB565) {
                rgb = picture.getImage(i);
                break;

            }

            else {
                opMode.telemetry.addLine("Didn't find correct RGB format");
                opMode.telemetry.update();


            }

        }

        Bitmap imageBitmap = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        imageBitmap.copyPixelsFromBuffer(rgb.getPixels());

        opMode.telemetry.addData("Image width", imageBitmap.getWidth());
        opMode.telemetry.addData("Image height", imageBitmap.getHeight());
        opMode.telemetry.update();


        picture.close();

        opMode.telemetry.addLine("Got bitmap");
        opMode.telemetry.update();

        return imageBitmap;
    }

    public String sample() throws InterruptedException{
        Bitmap bitmap = getBitmap();
        String bitmapCubePosition;

        ArrayList<Integer> xValues = new ArrayList<>();

        int avgX = 0;

        //top left = (0,0)
        for (int colNum = 0; colNum < bitmap.getWidth(); colNum +=2) {

            for (int rowNum = 0; rowNum < (int)(bitmap.getHeight() ); rowNum += 3) {
                int pixel = bitmap.getPixel(colNum, rowNum);

                int redPixel = red(pixel);
                int greenPixel = green(pixel);
                int bluePixel = blue(pixel);

                if (redPixel <= RED_THRESHOLD && greenPixel <= GREEN_THRESHOLD && bluePixel <= BLUE_THRESHOLD) {
                    xValues.add(colNum);

                }

            }

        }

        for (int x : xValues) {
            avgX+= x;
        }

        avgX /= xValues.size();

        if (avgX < (bitmap.getWidth() / 3.0)) {
            bitmapCubePosition = "left";

        }
        else if (avgX > (bitmap.getWidth() / 3.0) && avgX < (bitmap.getWidth() * 2.0/3)) {
            bitmapCubePosition = "center";

        }
        else {
            bitmapCubePosition = "right";

        }

        opMode.telemetry.addData("Cube Position", bitmapCubePosition);
        opMode.telemetry.update();
        return bitmapCubePosition;
    }
    public String findSkystones() throws InterruptedException{
        Bitmap bitmap = getBitmap();
        String bitmapCubePosition;

        int stone1 = bitmap.getPixel(100, 360);
        int redVal1 = red(stone1);

        int stone2 = bitmap.getPixel(527, 360);
        int redVal2 = red(stone2);

        int stone3 = bitmap.getPixel(954, 360);
        int redVal3 = red(stone3);

        ArrayList<Integer> vals = new ArrayList<Integer>();
        vals.add(redVal1);
        vals.add(redVal2);
        vals.add(redVal3);

        int min = Collections.min(vals);
        int pos = vals.indexOf(min);

        if (pos == 0){
            bitmapCubePosition = "left";
        }

        else if (pos == 1){
            bitmapCubePosition = "middle";
        }

        else if (pos == 2){
            bitmapCubePosition = "right";
        }
        else {
            bitmapCubePosition = "yikes";
        }

        return bitmapCubePosition;
    }

    public Bitmap vufConvertToBitmap(Frame frame) { return vuforia.convertFrameToBitmap(frame); }


}
