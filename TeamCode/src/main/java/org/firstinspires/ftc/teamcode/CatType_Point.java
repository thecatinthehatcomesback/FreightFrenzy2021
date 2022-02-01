package org.firstinspires.ftc.teamcode;


/**
 * @author Team #10273, The Cat in the Hat Comes Back.
 */
public class CatType_Point {

    // Attributes/Fields: //
    public double x;
    public double y;

    /**
     * Create a point and angle for coordinates and use in Pure Pursuit.
     *
     * @param x X-point on a cartesian coordinate plane.
     * @param y Y-point on a cartesian coordinate plane.
     */
    CatType_Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Turns the Point into a CurvePoint with a followDistance of 0 inches.
     *
     * @return a CurvePoint version of the Point.
     */
    public CatType_CurvePoint toCurvePoint() {
        return new CatType_CurvePoint(this.x, this.y, 0);
    }
}