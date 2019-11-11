package org.firstinspires.ftc.teamcode;


import android.graphics.Bitmap;

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
import java.util.Collections;
import java.util.concurrent.BlockingQueue;

import static android.graphics.Color.red;

@TeleOp(name="bitMapTests", group="Pushbot")
public class bitMapTests extends LinearOpMode {

    private VuforiaLocalizer vuforia;
    private Parameters parameters;
    private CameraDirection CAMERA_CHOICE = CameraDirection.BACK;

    private final int RED_THRESHOLD = 25;
    private final int GREEN_THRESHOLD = 25;
    private final int BLUE_THRESHOLD = 25;

    private final double widthFactor =  1280.0/3264;
    private final double heightFactor =  720.0/1836;

    private static final String VUFORIA_KEY = "Aad5SFz/////AAABmT+V+odOO0Lcr/j0iQ+cxkFwNC+TZ2mUCftLAt5wXY/BCUwwq4iU84o/Q15qndcAU2JPL+SN/qG+8GE9j0fDBHGkUHqAOBhu41XjysYMyF+kBeicTqdwfzaUzT5NLSvAU8aOZ+oIyQu+KFBoHVGylT7Yf6ASJw9gX34kck2ECReLzij/3gORcXvFFtXm/rGIyQGlxPGOXDv0ZnYmurQ79fH6fnArDP6Ylcc9QYQuPInhQ7BKzhiicPZhciwnKfZfa3CH4zu8zSWajDLDQmaj3iZxnkVsXac6XCk2cYMD37svmgGqI78M4bAcRUfOpPqGtqz+89jC8kUmCip8OSdmGO7ArKzBPHF3sctfMxurkuG4";

    private BlockingQueue<VuforiaLocalizer.CloseableFrame> frame;

    public static String bitmapSkyStonePosition;
    //WebcamName webcamName = null;

    @Override
    public void runOpMode() throws InterruptedException {

        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = this.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.hardwareMap.appContext.getPackageName());
        Parameters params = new Parameters(cameraMonitorViewId);


        params.vuforiaLicenseKey = VUFORIA_KEY;
        //params.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(params);


        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(4); //tells VuforiaLocalizer to only store one frame at a time
        vuforia.enableConvertFrameToBitmap();

        waitForStart();


        //getBitmap();
        //String pos =
        findBlueSkystones();
        //telemetry.addData("pos", pos);
        //telemetry.update();

        //sample();
        sleep(5000);
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

    public void sample() throws InterruptedException{
        Bitmap bitmap = getBitmap();
        sleep(2000);
        String bitmapCubePosition;

        ArrayList<Integer> xValues = new ArrayList<>();

        int avgX = 0;

        //top left = (0,0)
        //int colNum = 1280 * (1/5);
        int end = (int)(800 * widthFactor);

        for (int colNum = (int)(650 * widthFactor); colNum <= end; colNum++) {
            for(int rowNum = (int)(570 * heightFactor); rowNum < (int)(870 * heightFactor); rowNum++) {

                int stone1 = bitmap.getPixel(colNum, rowNum);
                int redVal1 = red(stone1);

                int stone2 = bitmap.getPixel(colNum + (int)(600 * widthFactor), rowNum);
                int redVal2 = red(stone2);

                int stone3 = bitmap.getPixel(colNum + (int)(1200 * widthFactor), rowNum);
                int redVal3 = red(stone3);

                if (redVal1 < redVal2 && redVal1 < redVal3) {
                    xValues.add(colNum);
                }
                else if (redVal2 < redVal1 && redVal2 < redVal3){
                    xValues.add(colNum + (int)(600 * widthFactor));
                }
                else {
                    xValues.add(colNum + (int)(1200 * widthFactor));
                }
                /*ArrayList<Integer> vals = new ArrayList<Integer>();
                vals.add(redVal1);
                vals.add(redVal2);
                vals.add(redVal3);

                int min = Collections.min(vals);
                int pos = vals.indexOf(min);

                if (pos == 0){
                    xValues.add(colNum);
                }

                else if (pos == 1){
                    xValues.add(colNum + 256);
                }

                else {
                    xValues.add(colNum + 512);
                }*/
            }
        }
        telemetry.addLine("test 1");
        telemetry.update();
        sleep(1000);

        for (int x : xValues) {
            avgX+= x;
        }

        telemetry.addData("avgx", avgX);
        telemetry.addLine("test 2");
        telemetry.update();
        sleep(1000);

        avgX /= xValues.size();

        if (avgX<= (1280 * 2/5)) {
            bitmapCubePosition = "left";

        }
        else if ((1280 * 2/5) < avgX && avgX <= (1280 * 3/5)) {
            bitmapCubePosition = "center";

        }

        else {
            bitmapCubePosition = "right";
        }

        telemetry.addData("width", bitmap.getWidth());
        telemetry.update();
        sleep(2000);



        telemetry.addData("Cube Position", bitmapCubePosition);
        telemetry.addData ("X-value", avgX);

        telemetry.update();
        sleep (10000);
        //return bitmapCubePosition;

    }

    public void colors() throws InterruptedException{
        Bitmap bitmap = getBitmap();
        String bitmapCubePosition;

        ArrayList<Integer> blueValues = new ArrayList<>();
        ArrayList<Integer> redValues = new ArrayList<>();
        ArrayList<Integer> greenValues = new ArrayList<>();
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

        double size2 = redValues.size();

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


        telemetry.addData("stone 1", redVal1);

        telemetry.addData("stone 2", redVal2);

        telemetry.addData("stone 3", redVal3);

        telemetry.addData("Cube Position", bitmapCubePosition);

        telemetry.update();
        sleep(20000);

    }

    public void coordinates() throws InterruptedException{
        Bitmap bitmap = getBitmap();

        telemetry.addData("upper left", red(bitmap.getPixel(10, 700)));
        telemetry.addData("upper right", red(bitmap.getPixel(1270, 700)));
        telemetry.addData("lower left", red(bitmap.getPixel(10, 20)));
        telemetry.addData("lower right", red(bitmap.getPixel(1270, 20)));
        telemetry.update();
        sleep(20000);

    }

    public void findRedSkystones() throws InterruptedException{
        Bitmap bitmap = getBitmap();
        String bitmapCubePosition;

        int stone1 = bitmap.getPixel((int)(790 * widthFactor), (int)(715 * heightFactor));//bitmap.getWidth() * 2/5, 20
        int redVal1 = red(stone1);

        int stone2 = bitmap.getPixel((int)(1390 * widthFactor), (int)(715 * heightFactor));//bitmap.getWidth()/2, 20
        int redVal2 = red(stone2);

        int stone3 = bitmap.getPixel((int)(2055 * widthFactor), (int)(715 * heightFactor));//bitmap.getWidth() * 3/5, 20
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
            bitmapCubePosition = "center";
        }

        else if (pos == 2){
            bitmapCubePosition = "right";
        }
        else {
            bitmapCubePosition = "yikes";
        }

        telemetry.addData("redval1", redVal1);
        telemetry.addData("redval2", redVal2);
        telemetry.addData("redval3", redVal3);
        telemetry.addData("left", vals.get(0));
        telemetry.addData("center", vals.get(1));
        telemetry.addData("right", vals.get(2));
        telemetry.update();
        sleep(5000);
        //return bitmapCubePosition;
    }
    public void findBlueSkystones() throws InterruptedException{
        Bitmap bitmap = getBitmap();
        String bitmapCubePosition;

        int stone1 = bitmap.getPixel((int)(1080 * widthFactor), (int)(715 * heightFactor));//bitmap.getWidth() * 2/5, 20
        int redVal1 = red(stone1);

        int stone2 = bitmap.getPixel((int)(1710 * widthFactor), (int)(715 * heightFactor));//bitmap.getWidth()/2, 20
        int redVal2 = red(stone2);

        int stone3 = bitmap.getPixel((int)(2340 * widthFactor), (int)(715 * heightFactor));//bitmap.getWidth() * 3/5, 20
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
            bitmapCubePosition = "center";
        }

        else if (pos == 2){
            bitmapCubePosition = "right";
        }
        else {
            bitmapCubePosition = "yikes";
        }

        telemetry.addData("redval1", redVal1);
        telemetry.addData("redval2", redVal2);
        telemetry.addData("redval3", redVal3);
        telemetry.addData("left", vals.get(0));
        telemetry.addData("center", vals.get(1));
        telemetry.addData("right", vals.get(2));
        telemetry.update();
        sleep(5000);
        //return bitmapCubePosition;
    }



    public Bitmap vufConvertToBitmap(Frame frame) { return vuforia.convertFrameToBitmap(frame); }


}
