package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.spartronics4915.lib.T265Camera;

public class CatHW_RealSense extends CatHW_Subsystem {
    /* Constructor */
    public CatHW_RealSense(CatHW_Async mainHardware){
        super(mainHardware);
    }
    private static T265Camera slamra = null;
    private boolean isCamStart;

    Rotation2d rotation;

    private double xPos;
    private double yPos;

    public void init() {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hwMap.appContext);

        }
        if(!isCamStart) {
            slamra.start();
            isCamStart = true;
        }

    }

    public boolean getCameraUpdate(){
        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return false;

        // We divide by 0.0254 to convert meters to inches
        xPos = -up.pose.getTranslation().getX() / 0.0254;
        yPos = -up.pose.getTranslation().getY() / 0.0254;

        rotation = up.pose.getRotation();
        return true;
    }

    public double getRotation(){
        return -rotation.getDegrees();
    }

    public double getXPos(){
        return xPos;
    }

    public double getYPos() {
        return yPos;
    }
}
