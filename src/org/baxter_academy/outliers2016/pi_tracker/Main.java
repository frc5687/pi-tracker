package org.baxter_academy.outliers2016.pi_tracker;

import edu.wpi.first.wpilibj.tables.ITable;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import java.security.Timestamp;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;


public class Main {

    public static void main(String[] args) {
        int camPort = 0;
        double exposure = .25;
        int team = 5687;
        boolean logging = false;
        boolean images = false;

        for (String arg : args) {
            String[] a = arg.toLowerCase().split("=");
            if (a.length == 2) {
                switch (a[0]) {
                    case "cam":
                    case "camera":
                    case "c":
                        try {
                            camPort = Integer.parseInt(a[1]);
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        break;
                    case "exposure":
                    case "exp":
                    case "e":
                        try {
                            exposure = Double.parseDouble(a[1]);
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        break;
                    case "team":
                        try {
                            team = Integer.parseInt(a[1]);
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        break;
                    case "logging":
                    case "logs":
                    case "log":
                    case "l":
                        try {
                            logging = a[1].equals("on") || a[1].equals("yes") || a[1].equals("true");
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        break;
                    case "images":
                    case "i":
                        try {
                            images = a[1].equals("on") || a[1].equals("yes") || a[1].equals("true");
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        break;
                }
            }
        }

        System.out.println(String.format("Camera port set to %1$d", camPort));
        System.out.println(String.format("Exposure set to %1$f", exposure));
        System.out.println(String.format("Team set to %1$d", team));
        System.out.println(String.format("Logging set to %1$b", logging));
        System.out.println(String.format("Images set to %1$b", images));

        // Initialize the NetworkTable library with team information
        NetworkTable.setClientMode();
        NetworkTable.setTeam(team);

        // We want the robot code to always look in the same place for output, so we use the same path as GRIP
        NetworkTable networkTable = NetworkTable.getTable("/PITracker");
        ITable inputs = null;

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
        camera.open(camPort); //Useless

        if (!camera.isOpened()) {
            System.out.println("Camera Error!");
        } else {
            System.out.println("Camera connected!");
        }

        // And set the exposure low (to improve contrast of retro-reflective tape)
        camera.set(15, exposure);

        long targetCenterX =-106;
        long targetWidth = 162;

        int lowerH = 50;
        int lowerL = 110;
        int lowerS = 64;

        int upperH = 94;
        int upperL = 235;
        int upperS = 255;

        int minArea = 20;

        if (networkTable.isConnected()) {
            networkTable.putNumber("inputs/HLS_LOWER_H", lowerH);
            networkTable.putNumber("inputs/HLS_LOWER_L", lowerL);
            networkTable.putNumber("inputs/HLS_LOWER_S", lowerS);

            networkTable.putNumber("inputs/HLS_UPPER_H", upperH);
            networkTable.putNumber("inputs/HLS_UPPER_L", upperL);
            networkTable.putNumber("inputs/HLS_UPPER_S", upperS);

            networkTable.putNumber("inputs/MIN_AREA", minArea);

            networkTable.putNumber("inputs/EXPOSURE", exposure);

            networkTable.putNumber("inputs/TARGET_WIDTH", targetWidth);
            networkTable.putNumber("inputs/TARGET_CENTERX", targetCenterX);
        }

        boolean first=true;
        Mat frame = new Mat();
        Mat hls = null;
        Mat filtered = null;
        Mat cont = null;
        if (networkTable.isConnected()) {
            inputs = networkTable.getSubTable("inputs");
        }

        while (true) {
            long mills = Instant.now().toEpochMilli() + 20;
            if (networkTable.isConnected() && inputs!=null) {
                inputs = networkTable.getSubTable("inputs");
                lowerH = (int) inputs.getNumber("HLS_LOWER_H", lowerH);
                lowerL = (int) inputs.getNumber("HLS_LOWER_L", lowerL);
                lowerS = (int) inputs.getNumber("HLS_LOWER_S", lowerS);

                upperH = (int) inputs.getNumber("HLS_UPPER_H", upperH);
                upperL = (int) inputs.getNumber("HLS_UPPER_L", upperL);
                upperS = (int) inputs.getNumber("HLS_UPPER_S", upperS);

                minArea = (int) inputs.getNumber("MIN_AREA", minArea);

                targetWidth = (long) inputs.getNumber("TARGET_WIDTH", targetWidth);
                targetCenterX = (long) inputs.getNumber("TARGET_CENTERX", targetCenterX);

                double newExposure = (double) inputs.getNumber("EXPOSURE", exposure);
                if (newExposure != exposure) {
                    if (logging) {
                        System.out.println(String.format("Resetting exposure to %1$f", newExposure));
                    }
                    camera.set(15, newExposure);
                    exposure = newExposure;
                }
            }


            // Capture a frame and write to disk
            camera.read(frame);
            if (images) {
                Imgcodecs.imwrite("1_bgr.png", frame);
            }

            if (first) {
                hls = new Mat(frame.size(), frame.type());
                filtered = new Mat(frame.size(), frame.type());
                cont = new Mat(frame.size(), frame.type());
                first = false;
            }

            // Convert to HLS color model
            Imgproc.cvtColor(frame, hls, Imgproc.COLOR_BGR2HLS);
            if (images) {
                Imgcodecs.imwrite("2_hls.png", hls);
            }

            // Filter using HLS lower and upper range
            Scalar lower = new Scalar(lowerH, lowerL, lowerS, 0);
            Scalar upper = new Scalar(upperH, upperL, upperS, 0);

            Core.inRange(hls, lower, upper, filtered);
            if (images) {
                Imgcodecs.imwrite("3_filtered.png", filtered);
            }

            // Find the contours...
            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Imgproc.findContours(filtered, contours, cont, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            if (images && contours.size()>0) {
                Imgcodecs.imwrite("4_cont.png", cont);
            }

            // Now find the biggest contour (if any)
            double maxArea = 0;
            MatOfPoint biggest = null;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea && area > minArea) {
                    maxArea = area;
                    biggest = contour;
                }
            }

            if (biggest != null) {
                // If we have one, find the bounding rectangle
                final Rect rect = Imgproc.boundingRect(biggest);

                // And its center point
                final double cx = rect.x - (frame.width() / 2);
                final double cy = rect.y - (frame.height() / 2);
                final int width = rect.width;

                if (networkTable.isConnected()) {
                    // Send it all to NetworkTables
                    networkTable.putString("TargetSighting", "Sighted");
                    networkTable.putNumber("width", width);
                    networkTable.putNumber("height", rect.height);
                    networkTable.putNumber("centerX", cx);
                    networkTable.putNumber("centerY", cy);
                    if (cx<targetCenterX) {
                        networkTable.putString("TargetCenter", "Left");
                    } else if (cx>targetCenterX) {
                        networkTable.putString("TargetCenter", "Right");
                    } else {
                        networkTable.putString("TargetCenter", "Centered");
                    }

                    if (width<targetWidth) {
                        networkTable.putString("TargetDistance", "Too far");
                    } else if (width>targetWidth) {
                        networkTable.putString("TargetDistance", "Too close");
                    } else {
                        networkTable.putString("TargetDistance", "In range");
                    }

                    if (logging) {
                        System.out.println(String.format("Height=%1$f, Width=%1$f, center=%2$f,%3$f", rect.size().height, rect.size().width, cx, cy));
                    }
                }
            } else {
                if (networkTable.isConnected()) {
                    // Send it all to NetworkTables
                    networkTable.putString("TargetSighting", "Absent");
                }
                if (logging) {
                    System.out.println(String.format("Target absent."));
                }

            }
            try {
                long w = mills - Instant.now().toEpochMilli();
                if (w > 0) {
                    Thread.sleep(mills - Instant.now().toEpochMilli());
                }
            } catch (Exception e) {
            }
        }

    }
}

