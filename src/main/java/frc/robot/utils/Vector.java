package frc.robot.utils;

// Import the needed libraries
import edu.wpi.first.math.geometry.Rotation2d;

public class Vector {
    
    // Private Variables
    private double x = 0.0;
    private double y = 0.0;
    private double mag = 0.0;
    private Rotation2d angle = Rotation2d.fromDegrees(0);

    /**
     * The constructor below will create a new vector
     * It calculates the angle and magnitude using the x and y comp
     * @param _x the x comp (inches)
     * @param _y the y comp (inches)
     */

     public Vector(double _x, double _y) {

        x = _x;
        y = _y;
        angle = new Rotation2d(Math.atan2(y, x));
        mag = Math.sqrt((Math.pow(x, 2)) + Math.pow(y, 2));

     }

    /**
     * The constructor below will create a new vector
     * This one is used when x and y are both 0
     * @param _x x comp
     * @param _y y comp
     * @param _angle angle if x&y are 0
     */

     public Vector (double _x, double _y, Rotation2d _angle) {

        x = _x;
        y = _y;

        if (Math.abs(x) < 1e-9 && Math.abs(y) < 1e-9) {
            angle = _angle;
        }
        else {
            angle = new Rotation2d(Math.atan2(y, x));
        }
        mag = Math.sqrt((Math.pow(x, 2)) + Math.pow(y, 2));
     }

    /**
     * Creates a vector with a magnitude and angle
     * It calculates the x and y
     * @param _mag the magnitude of the vector
     * @param _angle the angle of the vector
     */

     public Vector (double _mag, Rotation2d _angle) {

        mag = _mag;
        angle = _angle;
        angle = Calculations.NormalizeAngle(angle);
        x = mag * Math.cos(angle.getRadians());
        y = mag * Math.sin(angle.getRadians());

     }

    /**
     * The function below will subtract 2 vectors and calculate a new vector
     * @param v = the vector to subtract
     */

     public Vector SubtractVector (Vector v) {
        x -= v.GetXValue();
        y -= v.GetYValue();
        angle = new Rotation2d(Math.atan2(y, x));
        mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        return new Vector(x, y);
     }

    /**
     * Add Two vectors together
     * @param v
     */
    
     public Vector AddVector(Vector v) {
        x += v.GetXValue();
        y += v.GetYValue();
        angle = new Rotation2d(Math.atan2(y, x));
        mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        return new Vector(x, y);
     }

    /**
     * Change the frame of reference from robot to field oriented
     * @param GyroAngle Degrees
     */

     public Vector Transform(double GyroAngle) {
        angle = angle.plus(Rotation2d.fromDegrees(GyroAngle));
        x = mag * Math.cos(angle.getRadians());
        y = mag * Math.sin(angle.getRadians());
        return new Vector(x, y);
     }

      /**
       * Get the X comp in inches
       * @return x
       */
      public double GetXValue() {
        return x;
      }

      /**
       * Get the Y Comp in inches
       * @return y
       */
      public double GetYValue() {
        return y;
      }

      /**
       * Get the magnitude in inches
       * @return mag
       */
      public double GetMag() {
        return mag;
      }

      /**
       * Get the angle
       * @return
       */
      public Rotation2d GetAngle() {
        return angle;
      }

      /**
       * Set the X comp of a vector
       * @param _x x comp
       */
      public void SetX(double _x) {
        x = _x;
        angle = new Rotation2d(Math.atan2(y, x));
        mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
      }

      /**
       * Set the Y Comp of a vector
       * @param _y y comp
       */
       public void SetY(double _y) {
        y = _y;
        angle = new Rotation2d(Math.atan2(y, x));
        mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
       }

       /**
        * Set the Magnitude
        * @param _mag (inches) magnitude to be set
        */
        public void SetMag(double _mag) {
            x = _mag * Math.cos(angle.getRadians());
            y = _mag * Math.sin(angle.getRadians());
            mag = _mag;
        }

        /**
         * Set the angle of the vector
         * @param _angle
         */

         public void SetAngle(Rotation2d _angle) {
            x = mag * Math.cos(_angle.getRadians());
            y = mag * Math.sin(_angle.getRadians());
            angle = _angle;
         }

    /**
     * This function will reflect the vector on the y axis
     */

     public void ReflectY() {
        x = -x;
        angle = new Rotation2d(Math.atan2(Math.sin(angle.getRadians()), -Math.cos(angle.getRadians())));
     }

    /**
     * Prints the vector with a prefix string
     * @param pre the string to preface the print
     */
    public void print(String pre) {
        System.out.printf("%s: x:%.3f y:%.3f a:%.3f\n", pre, x, y, angle.getDegrees());
    }

    /**
     * Prints the vector without a prefix
     */
    public void print() {
        System.out.printf("x:%.3f y:%.3f a:%.3f\n", x, y, angle.getDegrees());
    }

    Vector() {}

}
