package org.baxter_academy.outliers2016.pi_tracker;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

/**
 * Proxy class for sending data to and receiving data from the roborio
 * Created by Ben Bernard on 6/17/2016.
 */
public class RobotProxy {
    private long robotTimestamp = 0;
    private boolean ringlighton = false;
    private InetAddress robotAddress;
    DatagramSocket outgoingSocket;

    public static final int piPort = 27002;
    public static final int rioPort = 27001;

    private RobotThread robotThread;

    public RobotProxy(String robotAddress) {
        try {
            this.robotAddress = InetAddress.getByName(robotAddress);
            outgoingSocket = new DatagramSocket();
        } catch (Exception e) {
        }

        robotThread = new RobotThread(this);

        new Thread(robotThread).start();

    }

    public void Send(long rioMillis, boolean isSighted, double offsetAngle, double distance) {
        if (robotAddress!=null) {
            StringBuilder buffer = new StringBuilder();
            buffer.append(Long.toString(rioMillis));
            buffer.append(";");

            buffer.append(Boolean.toString(isSighted));
            buffer.append(";");

            buffer.append(Double.toString(offsetAngle));
            buffer.append(";");

            buffer.append(Double.toString(distance));
            buffer.append(";");

            byte[] sendData = new byte[1024];
            sendData = buffer.toString().getBytes();
            DatagramPacket sendPacket = new DatagramPacket(sendData, buffer.length(), robotAddress, rioPort);
            try {
                outgoingSocket.send(sendPacket);
            } catch (IOException ioe) {
            };
        }

    }

    public synchronized boolean isRinglighton() {
        return ringlighton;
    }

    public synchronized long getRobotTimestamp() {
        return robotTimestamp;
    }

    protected synchronized void setRobotTimestamp(long timestamp) {
        this.robotTimestamp = timestamp;
    }

    protected synchronized void setRinglighton(boolean ringlighton) {
        this.ringlighton = ringlighton;
    }

    public class RobotThread implements Runnable {
        private RobotProxy robot;

        public RobotThread(RobotProxy robot) {
            this.robot = robot;
        }

        @Override
        public void run() {
            // Create the listener
            DatagramSocket incomingSocket;
            byte[] receiveData = new byte[1024];
            try {
                incomingSocket = new DatagramSocket(piPort);
                while (true) {
                    DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
                    incomingSocket.receive(receivePacket);
                    String raw = new String(receivePacket.getData());

                    synchronized (this) {
                        String[] a = raw.split(";");
                        long millis = Long.parseLong(a[0]);
                        boolean ringlighton = Boolean.parseBoolean(a[1]);
                        robot.setRobotTimestamp(millis);
                        robot.setRinglighton(ringlighton);
                    }
                }

            } catch (IOException ioe) {

            }
        }
    }

}
