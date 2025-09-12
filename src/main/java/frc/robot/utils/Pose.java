package frc.robot.utils;

public class Pose {
    
   // Private variables
   private double x; 
   private double y; 
   private double angle; 

    /**
     * Create a new pose
     * @param _x x comp
     * @param _y y comp
     * @param _angle angle of the pose
     */

     public Pose(double _x, double _y, double _angle) {
        x = _x;
        y = _y;
        angle = _angle;  // FIXED: Don't wrap on construction, store raw angle
     }

    /**
     * Create a new pose at 0 deg (like C++ version)
     */

     public Pose() {
        x = 0;
        y = 0;
        angle = 0;  // FIXED: Changed from 90 to 0 to match C++ default
     }

     public Pose (Pose other) {
      this.x = other.x;
      this.y = other.y;
      this.angle = other.angle;
     }

    /**
     * Get the x comp in inches
     * @return x comp
     */

     public double GetXValue() {
        return x;
     }

    /**
     * Get the y comp in inches
     * @return y comp
     */

     public double GetYValue() {
        return y;
     }

    /**
     * Get the angle in degrees - RAW angle without wrapping
     * @return angle in degrees
     */

     public double GetAngleValue() {
        return angle;  // FIXED: Return raw angle like C++ version
     }

    /**
     * Get the angle wrapped to [-180, 180] for display/comparison purposes
     * @return wrapped angle in degrees
     */
     public double GetWrappedAngle() {
        return wrapTo180(angle);
     }

    /**
     * Set the x componenet in inches
     * @param _x x comp
     */

     public void SetX(double _x) {
        x = _x;
     }

    /**
     * Set the y componenet in inches
     * @param _y y comp
     */

     public void SetY(double _y) {
        y = _y;
     }

    /**
     * Set the angle in degrees - stores raw angle
     * @param _angle new angle
     */

     public void SetAngle(double _angle) {
        angle = _angle;  // FIXED: Store raw angle like C++
     }

    /**
     * Reflect the pose along the y axis
     */

     public void reflectY() {
      this.x = -this.x;
      this.angle = 180 - this.angle;
      this.angle = Calculations.NormalizeAngle(this.angle);
     }

    /**
     * Add vector to the pose
     * @param v the vector to add
     */

     public void AddVector(Vector v) {
        x += v.GetXValue();
        y += v.GetYValue();
     }

    /**
     * Subtract vector from the pose
     * @param v the vector to subtract
     */

     public void SubtractVector(Vector v) {
        x -= v.GetXValue();
        y -= v.GetYValue();
     }

    /**
     * Subtract a pose from another pose
     * @param p The pose to subtract
     * @return the resultant vector
     */

     public Vector Subtract(Pose p) {
        return new Vector(x - p.GetXValue(), y - p.GetYValue());
     }

    /**
     * Sets the x y and angle comps
     * @param _x new x comp
     * @param _y new y comp
     * @param _angle new angle comp
     */

     public void SetPose(double _x, double _y, double _angle) {
        x = _x;
        y = _y;
        angle = _angle;  // FIXED: Store raw angle
     }

    /**
     * Sets the x y and angle componenets
     * @param _pose new pose
     */

     public void SetPose(Pose other) {
        x = other.GetXValue();
        y = other.GetYValue();
        angle = other.GetAngleValue();  // FIXED: Use raw angle
     }

     // Keep the wrapping function for internal use when needed
     private double wrapTo180(double angle) {
        while (angle <= -180) angle += 360;
        while (angle > 180) angle -= 360;
        return angle;
     }

    /**
     * Change the frame of reference from robot to field
     * @param GyroAngle the angle of the gyro as a double, but in degrees
     */

     public void Transform(double GyroAngle) {
        angle = angle + GyroAngle;
        // FIXED: Don't automatically normalize - let the caller decide
     }

    /**
     * Print the pose with a prefix string
     * @param pre String to print before the pose
     */
    public void Print(String pre) {
        System.out.printf("%s: x:%.2f y:%.2f a:%.2f\n", pre, x, y, angle);
    }

    /**
     * Print the pose with a prefix and a number
     * @param pre String to print before the pose
     * @param i   Number to print before the pose
     */
    public void Print(String pre, int i) {
        System.out.printf("%s %d: x:%.2f y:%.2f a:%.2f\n", pre, i, x, y, angle);
    }

    /**
     * Print the pose with a number only
     * @param i Number to print before the pose
     */
    public void Print(int i) {
        System.out.printf("%d: x:%.2f y:%.2f a:%.2f\n", i, x, y, angle);
    }

    /**
     * Print the pose without any prefix
     */
    public void Print() {
        System.out.printf("x:%.2f y:%.2f a:%.2f\n", x, y, angle);
    }
     
}