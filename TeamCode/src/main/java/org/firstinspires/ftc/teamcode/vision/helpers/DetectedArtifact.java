package org.firstinspires.ftc.teamcode.vision.helpers;

import org.opencv.core.Point;

public class DetectedArtifact {
    public final Point center;
    public final double area;
    public final double width;
    public final double height;

    public DetectedArtifact(Point center, double area, double width, double height) {
        this.center = center;
        this.area = area;
        this.width = width;
        this.height = height;
    }


}
