package org.baxter_academy.outliers2016.pi_tracker;

import com.sun.org.apache.xerces.internal.impl.xpath.regex.Match;
import edu.wpi.first.wpilibj.tables.ITable;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileDescriptor;
import java.io.FileWriter;
import java.net.*;
import java.security.Timestamp;
import java.text.SimpleDateFormat;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Date;
import java.util.Enumeration;
import java.util.List;


public class Main {
    private static double CENTER_X = 319.5;
    private static double CENTER_Y = 239.5;
    private static double FOCAL = 543.25;

    private static double CAMERA_HEIGHT = 10;
    private static double TARGET_HEIGHT = 91;
    private static double CAMERA_ANGLE = 45.00;


    public static void main(String[] args) {
        int camPort = 0;
        double exposure = .25;
        int team = 5687;
        boolean logging = false;
        boolean images = false;
        int rX = 0;
        int rY = 0;

        long startMills = Instant.now().toEpochMilli();

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
        NetworkTable tracking = NetworkTable.getTable("PITracker/tracking");
        NetworkTable inputs = NetworkTable.getTable("PITracker/inputs");
        NetworkTable dashboard = NetworkTable.getTable("SmartDashboard");

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


        long folderNumber = 1;

        String prefix = "./";
        if (images) {
            while (true) {
                prefix = "/home/pi/images/" + folderNumber;
                File imageDir = new File(prefix);
                if (!imageDir.exists()) {
                    imageDir.mkdir();
                    if (imageDir.exists()) {
                        prefix += "/";
                    } else {
                        prefix = "./";
                    }
                    break;
                }
                folderNumber++;
            }
        }

        /*
        int retry = 240;
        while (!tracking.isConnected() && retry > 0) {
            retry--;
            try {
                Thread.sleep(1000);
            } catch (InterruptedException ie) {
                break;
            }
        }
        */

        if (tracking.isConnected()) {
            // Send it all to NetworkTables
            try {
                tracking.putString("Address", getFirstNonLoopbackAddress(true, false).getHostAddress());
            } catch (Exception e) {
            }
            tracking.putNumber("Folder", folderNumber);
            tracking.putNumber("Start Time: ", startMills);
        }
        // tracking.putString("Test2", "Show you!!!!!");
        // tracking.setPersistent("Test2");

        // if (tracking.isConnected()) {
        //     inputs = tracking.getSubTable("inputs");
        //}




        int lowerH = 50;
        int lowerL = 34;
        int lowerS = 163;

        int upperH = 94;
        int upperL = 220;
        int upperS = 255;

        int minArea = 20;

        if (inputs.isConnected() && !inputs.containsKey("TARGET_WIDTH")) {
            inputs.putNumber("HLS_LOWER_H", lowerH);
            inputs.putNumber("HLS_LOWER_L", lowerL);
            inputs.putNumber("HLS_LOWER_S", lowerS);

            inputs.putNumber("HLS_UPPER_H", upperH);
            inputs.putNumber("HLS_UPPER_L", upperL);
            inputs.putNumber("HLS_UPPER_S", upperS);

            inputs.putNumber("MIN_AREA", minArea);

            inputs.setPersistent("HLS_LOWER_H");
            inputs.setPersistent("HLS_LOWER_L");
            inputs.setPersistent("HLS_LOWER_S");
            inputs.setPersistent("HLS_UPPER_H");
            inputs.setPersistent("HLS_UPPER_L");
            inputs.setPersistent("HLS_UPPER_S");

            inputs.setPersistent("MIN_AREA");
        }

        boolean first=true;
        Mat frame = new Mat();
        Mat hls = null;
        Mat filtered = null;
        Mat cont = null;
        MatOfInt minCompressionParam = new MatOfInt(Imgcodecs.CV_IMWRITE_PNG_COMPRESSION, 3);
        MatOfInt compressionParam = new MatOfInt(Imgcodecs.CV_IMWRITE_PNG_COMPRESSION, 6);
        MatOfInt maxCompressionParam = new MatOfInt(Imgcodecs.CV_IMWRITE_PNG_COMPRESSION, 8);

        while (true) {
            long mills = Instant.now().toEpochMilli() - startMills;
            if (inputs.isConnected()) {
                lowerH = (int) inputs.getNumber("HLS_LOWER_H", lowerH);
                lowerL = (int) inputs.getNumber("HLS_LOWER_L", lowerL);
                lowerS = (int) inputs.getNumber("HLS_LOWER_S", lowerS);

                upperH = (int) inputs.getNumber("HLS_UPPER_H", upperH);
                upperL = (int) inputs.getNumber("HLS_UPPER_L", upperL);
                upperS = (int) inputs.getNumber("HLS_UPPER_S", upperS);

                minArea = (int) inputs.getNumber("MIN_AREA", minArea);

                double newExposure = (double) inputs.getNumber("EXPOSURE", exposure);
                if (newExposure != exposure) {
                    if (logging) {
                        System.out.println(String.format("Resetting exposure to %1$f", newExposure));
                    }
                    camera.set(15, newExposure);
                    exposure = newExposure;
                }
            }
            if (dashboard.isConnected()) {
                images = dashboard.getBoolean("lights/ringlight", false);
            } else {
                images = true; // false;
            }


            // Capture a frame and write to disk
            camera.read(frame);
            rX = frame.width();
            rY = frame.height();

            if (images) {
                Imgcodecs.imwrite(prefix + "a_bgr_" + mills + ".png", frame, minCompressionParam);
            }

            if (first) {
                hls = new Mat(frame.size(), frame.type());
                filtered = new Mat(frame.size(), frame.type());
                cont = new Mat(frame.size(), frame.type());
                first = false;
            }

            // Convert to HLS color model
            Imgproc.cvtColor(frame, hls, Imgproc.COLOR_BGR2HLS);
            //if (images) {
            //    Imgcodecs.imwrite(prefix + "b_hls_" + mills + ".png", hls, maxCompressionParam);
            //}

            // Filter using HLS lower and upper range
            Scalar lower = new Scalar(lowerH, lowerL, lowerS, 0);
            Scalar upper = new Scalar(upperH, upperL, upperS, 0);

            Core.inRange(hls, lower, upper, filtered);
            if (images) {
                Imgcodecs.imwrite(prefix + "c_flt_" + mills + ".png", filtered, minCompressionParam);
            }

            // Find the contours...
            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Imgproc.findContours(filtered, contours, cont, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

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
                final double cx = rect.x + (rect.width / 2);
                final double cy = rect.y + (rect.height / 2);


                final int width = rect.width;
                final int height = rect.height;

                // Find the lateral offset angle
                final double offsetAngle = getAngle(cx, CENTER_X);


                // Now find the vertical angle
                final double verticalAngle = getAngle(cy, CENTER_Y);

                // From that, determine the distance to the target
                final double distance = getDistance(verticalAngle);


                if (tracking.isConnected()) {
                    // Send it all to NetworkTables
                    tracking.putNumber("Mills", mills);
                    tracking.putBoolean("TargetSighted", true);
                    tracking.putString("TargetSighting", "Sighted");
                    tracking.putNumber("width", width);
                    tracking.putNumber("height", rect.height);
                    tracking.putNumber("centerX", cx);
                    tracking.putNumber("centerY", cy);
                    tracking.putNumber("offsetAngle", offsetAngle);
                    tracking.putNumber("distance", distance);

                    if (logging) {
                        System.out.println(String.format("Height=%1$f, Width=%1$f, Center=%2$f,%3$f, Angle=%4$f, Distance=%5$f", rect.size().height, rect.size().width, cx, cy, offsetAngle, distance));
                    }

                    if (images) {
                        try {
                            //create a temporary file
                            File logFile=new File(prefix + "d_log_" + mills + ".txt");

                            BufferedWriter writer = new BufferedWriter(new FileWriter(logFile));
                            writer.write("");
                            writer.write("Mills: " + mills); writer.newLine();
                            writer.write("TargetSighted: true"); writer.newLine();
                            writer.write("TargetSighting: Sighted"); writer.newLine();
                            writer.write("width: " + width); writer.newLine();
                            writer.write("height: " + rect.height); writer.newLine();
                            writer.write("centerX: " + cx); writer.newLine();
                            writer.write("centerY: " + cy); writer.newLine();
                            writer.write("offsetAngle: " + offsetAngle); writer.newLine();
                            writer.write("distance: " + distance); writer.newLine();
                            //Close writer
                            writer.close();
                        } catch(Exception e) {

                        }
                    }
                }
            } else {
                if (tracking.isConnected()) {
                    // Send it all to NetworkTables
                    tracking.putBoolean("TargetSighted", false);
                    tracking.putString("TargetSighting", "Absent");
                }
                if (logging) {
                    System.out.println(String.format("Target absent."));
                }

                if (images) {
                    try {
                        //create a temporary file
                        File logFile=new File(prefix + "d_log_" + mills + ".txt");

                        BufferedWriter writer = new BufferedWriter(new FileWriter(logFile));
                        writer.write("");
                        writer.write("Mills: " + mills); writer.newLine();
                        writer.write("TargetSighted: false"); writer.newLine();
                        writer.write("TargetSighting: Absent"); writer.newLine();
                        //Close writer
                        writer.close();
                    } catch(Exception e) {

                    }
                }

            }


            try {
                // Make sure that this has taken AT LEAST 20 milliseconds.
                // If not, sleep until 20ms have passed
                long w = (Instant.now().toEpochMilli() - mills);
                if (w < 20) {
                    Thread.sleep(20 - w);
                }
            } catch (Exception e) {
            }
        }

    }


    private static double getAngle(double centerX, double frameCenter) {
        return Math.atan((centerX - frameCenter)/FOCAL);
    }

    private static double getDistance(double verticalAngle) {
        return TARGET_HEIGHT - CAMERA_HEIGHT / Math.tan((verticalAngle + CAMERA_ANGLE)* Math.PI / 180);
    }


    private static InetAddress getFirstNonLoopbackAddress(boolean preferIpv4, boolean preferIPv6) throws SocketException {
        Enumeration en = NetworkInterface.getNetworkInterfaces();
        while (en.hasMoreElements()) {
            NetworkInterface i = (NetworkInterface) en.nextElement();
            for (Enumeration en2 = i.getInetAddresses(); en2.hasMoreElements(); ) {
                InetAddress addr = (InetAddress) en2.nextElement();
                if (!addr.isLoopbackAddress()) {
                    if (addr instanceof Inet4Address) {
                        if (preferIPv6) {
                            continue;
                        }
                        return addr;
                    }
                    if (addr instanceof Inet6Address) {
                        if (preferIpv4) {
                            continue;
                        }
                        return addr;
                    }
                }
            }
        }
        return null;
    }
}

