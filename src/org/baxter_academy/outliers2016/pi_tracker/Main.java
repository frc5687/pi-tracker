package org.baxter_academy.outliers2016.pi_tracker;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import java.util.ArrayList;
import java.util.List;


public class Main {

    public static void main(String[] args) {

        // Initialize the NetworkTable library with team information
        NetworkTable.setClientMode();
        NetworkTable.setTeam(5687);

        // We want the robot code to always look in the same place for output, so we use the same path as GRIP
        NetworkTable networkTable = NetworkTable.getTable("/GRIP/tracking");

        // Initialize OpenCV
        System.out.println("Loading OpenCV...");
        // Load the native library.
        System.loadLibrary("opencv_java310");

        System.out.println("Initializing camera...");
        VideoCapture camera = new VideoCapture(0);
        try {
            Thread.sleep(1000);
        } catch (Exception e) {
        }

        // Open the camera
        camera.open(0); //Useless

        if(!camera.isOpened()){
            System.out.println("Camera Error!");
        }
        else{
            System.out.println("Camera connected!");
        }

        // And set the exposure low (to improve contrast of retro-reflective tape)
        //camera.set(15, -11);


        while (true) {
            Mat frame = new Mat();
            Mat hls = new Mat();
            Mat filtered = new Mat();
            Mat cont = new Mat();

            // Capture a frame and write to disk
            camera.read(frame);
            Imgcodecs.imwrite("1.png", frame);

            // Convert to HLS color model
            Imgproc.cvtColor(frame, hls, Imgproc.COLOR_BGR2HLS);
            Imgcodecs.imwrite("2.png", hls);

            // Filter using HLS lower and upper range
            Scalar lower = new Scalar(36, 73, 44);
            Scalar upper = new Scalar(83, 211, 188);
            Core.inRange(hls, lower, upper, filtered);
            Imgcodecs.imwrite("3.png", filtered);

            // Find the contours...
            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Imgproc.findContours(filtered, contours, cont, Imgproc.RETR_EXTERNAL,  Imgproc.CHAIN_APPROX_SIMPLE);
            Imgcodecs.imwrite("4.png", cont);

            // Now find the biggest contour (if any)
            double maxArea = 0;
            MatOfPoint biggest = null;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    biggest = contour;
                }
            }

            if (biggest!=null) {
                // If we have one, find the bounding rectangle
                final Rect rect = Imgproc.boundingRect(biggest);

                // And its center point
                final double cx = rect.x - (frame.width() / 2);
                final double cy = rect.y - (frame.height() / 2);

                // Send it all to NetworkTables
                networkTable.putNumber("width", rect.width);
                networkTable.putNumber("height", rect.height);
                networkTable.putNumber("centerX", cx);
                networkTable.putNumber("centerY", cy);

                // And log
                System.out.println(String.format("Width=%1$f, center=%2$f,%3$f", rect.size().width, cx, cy));
            }
        }
    }
}
