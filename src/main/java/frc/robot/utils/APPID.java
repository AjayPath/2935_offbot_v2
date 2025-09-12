package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

public class APPID {

    // Private variables used throughout the PID
    private double m_p; // Proportional Coefficient
    private double m_i; // Integral Coefficient
    private double m_d; // Derivative Coefficient
    
    private double m_izone; // Integral Zone range -> prevent integral windup
    
    private double m_desiredValue; // Desired Value that we want to reach
    private double m_oldDesiredValue; // Value to check if the setpoint has been changed
    private double m_previousValue; // The last called value
    private double m_errorSum; // Sum of the previous errors (used in the calc for I)
    private double m_errorIncrement; // Max Increment to error sum during each call
    private double m_errorEpsilon; // THe allowable error in determining target is reached 

    private boolean m_firstCycle; // Flage to identify first cycle
    private double m_maxOutput; // Ceiling on calcuating the output

    private int m_minCycleCount; // Minimum number of cycles in epsilon range to be done
    private int m_cycleCount; // Current number of cycles in epsilon range

    private final Timer pidTimer; // PID Timer
    private static final double FIXED_DT = 0.02; // Fixed 20ms loop time

    /**
     * The constructor below will initialize the PID Controller
     * using the inputed values and a standard predetermined set
     * @param p p term
     * @param i i term
     * @param d d term
     * @param epsilon error term
     */

    public APPID(double p, double i, double d, double epsilon) {

        // Inputted Values
        m_p = p;
        m_i = i;
        m_d = d;
        m_errorEpsilon = epsilon;

        // Standard Values
        m_desiredValue = 0;
        m_oldDesiredValue = 0;
        m_previousValue = 0;
        m_firstCycle = true;
        m_maxOutput = 1.0;
        m_errorSum = 0;
        m_errorIncrement = 1;
        m_izone = 0;

        // Trackers
        m_cycleCount = 0;
        m_minCycleCount = 10;   // default value

        // Initialize Timers
        pidTimer = new Timer();
        pidTimer.start();
        pidTimer.reset();

    }

    /**
     * Set the PID terms
     * @param p
     * @param i
     * @param d
     */
    public void setConstants(double p, double i, double d) {

        m_p = p;
        m_i = i;
        m_d = d;
        
    }

    /**
     * Set the Izone paramenter
     * @param izone izone value
     */
    public void setIzone(double izone) {
        m_izone = izone;
    }    

    /**
     * Set the epsilon error
     * @param epsilon error
     */
    public void setErrorEpsilon(double epsilon) {
        m_errorEpsilon = epsilon;
    }

    /**
     * Set the error increment
     * @param inc error increment term
     */
    public void setErrorIncrement(double inc) {
        m_errorIncrement = inc;
    }

    /**
     * Set the target value
     * @param target desired value
     */
    public void setDesiredValue(double target) {
        m_desiredValue = target;
    }

    public void setMaxOutput (double max) {
        if (max >= 0.0 && max <= 1.0) {
            m_maxOutput = max;
        }
    }

    /**
     * Resets the error sum back to zero
     */
    public void resetErrorSum() {
        m_errorSum = 0;
    }

    /**
     * This function is the crux of the PID Controller
     * It calculates teh PID value based on its current value
     * @param currentValue
     */

    public double calcPID(double currentValue) {

        // Initialize all componenets to start a 0
        double pVal = 0.0;
        double iVal = 0.0;
        double dVal = 0.0;

        // For the first cycle, do not apply a d term
        if (m_firstCycle) {
            m_previousValue = currentValue; // Basicallt the velocity is 0
            m_firstCycle = false; // Set flag to false now
            pidTimer.reset(); // Reset the timer
        }

        // If the setpoint has been changed, reset some states
        if (m_oldDesiredValue != m_desiredValue) {
            m_firstCycle = true;
            m_errorSum = 0; // Reset integral when setpoint changes
        }

        // The section below calculates the P component
        double error = m_desiredValue - currentValue;
        pVal = m_p * error;

        // The section below calculates the I component
        // FIXED: Simplified integral logic to be more symmetric
        
        if (Math.abs(error) > m_errorEpsilon) {
            // Only accumulate error if outside epsilon band
            if (m_izone == 0 || Math.abs(error) <= m_izone) {
                // Add error with increment limiting
                if (Math.abs(error) <= m_errorIncrement) {
                    m_errorSum += error;
                } else {
                    // Clamp to increment size but preserve sign
                    m_errorSum += Math.signum(error) * m_errorIncrement;
                }
            }
        } else {
            // Inside epsilon band - zero out integral
            m_errorSum = 0;
        }

        // Calculate the iVal
        iVal = m_i * m_errorSum;

        // Calculate the D component with fixed timestep for consistency
        double dt = pidTimer.get();
        if (dt > 0 && !m_firstCycle) {
            double velocity = (currentValue - m_previousValue) / dt;
            dVal = m_d * velocity;
        } else {
            dVal = 0;
        }

        // Calculate and Limit the Output
        // Output = P + I - D (note the minus for D term)
        double output = pVal + iVal - dVal;

        // Symmetric clamping
        if (output > m_maxOutput) {
            output = m_maxOutput;
        } else if (output < -m_maxOutput) {
            output = -m_maxOutput;
        }

        // Save the current value for the next cycles D value calc
        m_previousValue = currentValue;
        pidTimer.reset();
        m_oldDesiredValue = m_desiredValue;
        
        // Return the output
        return output;
    }

    /**
     * This will set the number of minimum cycles that the value must stay at
     * @param n number of cycles
     */
    public void setMinDoneCycles (int n) {
        m_minCycleCount = n;
    }

    public boolean isDone() {
        double currentError = Math.abs(m_desiredValue - m_previousValue);
        
        if (currentError <= m_errorEpsilon && !m_firstCycle) {
            m_cycleCount++;
            if (m_cycleCount >= m_minCycleCount) {
                m_cycleCount = 0;
                return true;
            }
        } else {
            m_cycleCount = 0; // Reset counter if we leave epsilon range
        }
        return false;
    }

    public void reset() {
        m_errorSum = 0;
        m_previousValue = 0;
        m_firstCycle = true;
        m_cycleCount = 0;
        pidTimer.reset();
    }
    
}