// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This class is a part of the utils folder
package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * This Class is used to group simple mathematical functions.
 * This calculations are use for vector math.
 */
public class Calculations {

    /**
     * SignSquare will square the input value and keep the sign
     * @param input the value to be squared
     * @return The result of the computation
     */

     public static double SignSquare (double input) {
        // Check if the input is less then 0
        if (input < 0) {
            // return the square but with a negative
            return (-input * input);
        }
        else {
            // return the square 
            return (input * input);
        }
     }

    /**
     * NormalizeAngle will return the given angle from -180 to 180
     * @param input the angle to normalize, it will be of type Rotation 2D which is degrees
     * @return the input with the change
     */

     public static Rotation2d NormalizeAngle (Rotation2d input) {
        // Check if the input is less then -180
        while (input.getDegrees() < -180) {
            // In java the object cannot be changed
            // So we create a new object with the change we want
            input = input.plus(Rotation2d.fromDegrees(360));
        }
        while (input.getDegrees() > 180) {
            input = input.minus(Rotation2d.fromDegrees(360));
        }
        return input;
     }

    /**
     * LimitOutput is a function that will limit the inputted value
     * @param input number to limit
     * @param LIMIT the limit
     * @return the value if not above the limit, otherwise returns the limit
     */

     public static double LimitOutput (double input, double LIMIT) {
        // Check if the input is greater then the limit
        if (input > LIMIT) {
            return LIMIT;
        }
        // Check if the input is less then then negative limit
        else if (input < -LIMIT) {
            return (-LIMIT);
        }
        // Return this if we are not over the limit
        else {
            return input;
        }
     }

    /**
     * The pyth functions are used to solve pythagorem theorem
     * They are used to find the Euclidean Distance of a vector
     * Euclidean Distance = Length + Magnitude of a 2D Vector
     * @param x X Value
     * @param y Y Value
     * @return the magnitude
     */

     public static double pyth (double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
     }

     public static double pyth (int x, int y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
     }

    /**
     * Max Returns the max of 2 numbers
     * @param num1
     * @param num2
     * @return double of the biggest number
     */

     public static double Max (double num1, double num2) {
        if (num1 > num2) {
            return num1;
        }
        else {
            return num2;
        }
     }

    /**
     * Get sign will return the sign of the number
     * @param value is the number we want to check
     * @return returns a double value of negative or positive
     */

     public static double GetSign (double value) {
        if (value >= 0.0) {
            return 1;
        } 
        else {
            return -1;
        }
     }

     public static double signedAngleDifference(double targetDegrees, double currentDegrees) {
        double diff = targetDegrees - currentDegrees;
    
        // Wrap difference into [-180, 180]
        while (diff > 180) diff -= 360;
        while (diff <= -180) diff += 360;
    
        return diff;
    }

    /**
     * Normalize an angle in degrees to the range (-180, 180].
     * @param angleDeg The angle in degrees
     * @return normalized angle in degrees
     */
    public static double NormalizeAngle(double angleDeg) {
        double result = angleDeg % 360.0;
        if (result <= -180.0) {
            result += 360.0;
        } else if (result > 180.0) {
            result -= 360.0;
        }
        return result;
    }

    /**
     * Normalize an angle in degrees to the range [0, 360).
     * @param angleDeg The angle in degrees
     * @return normalized angle in degrees between 0 and 360
     */
    public static double NormalizeAngle360(double angleDeg) {
        double result = angleDeg % 360.0;
        if (result < 0.0) {
            result += 360.0;
        }
        return result;
    }

    /**
     * Calculate the shortest angular distance between two angles in 0-360 range
     * @param targetDegrees Target angle (0-360)
     * @param currentDegrees Current angle (0-360) 
     * @return Shortest angular distance (-180 to 180), positive = CCW, negative = CW
     */
    public static double shortestAngularDistance(double targetDegrees, double currentDegrees) {
        // Ensure both angles are in 0-360 range
        double target = NormalizeAngle360(targetDegrees);
        double current = NormalizeAngle360(currentDegrees);
        
        double diff = target - current;
        
        // Convert to shortest path
        if (diff > 180.0) {
            diff -= 360.0;
        } else if (diff < -180.0) {
            diff += 360.0;
        }
        
        return diff;
    }
}