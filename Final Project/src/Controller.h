/** @file first_order_IIR.h
 *    This file contains the headers for a class that implements a first-order
 *    infinite impulse response digital filter as a demonstration.
 * 
 *  @author JR Ridgely
 *  @date  2020-Oct-18 Original file
 */


/** @brief   Class which implements a first-order IIR low-pass digital filter.
 *  @details This filter is simple enough that it doesn't really need a class
 *           to implement conveniently, but it does provide an opportunity to
 *           show how the features of a class work.
 */
class Controller
{
protected:
    /// The value of the filter output, which is saved between runs
    float filter_output;

    /// The filter coefficient @b A, which is the feedback gain
    float A_coeff;

    /// The filter coefficient @b B, which added to @b A makes unity
    float B_coeff;

public:
    // Constructor which is given ont filter coefficient
    Controller (uint16_t motor_pwr, uint16_t encA, uint16_t encB, uint16_t SDA, uint16_t SDL);

    void PID (void);
    void PWM (void);
    void Calculator(void);
    void Encoder(void);
    void IMU(void);


};


