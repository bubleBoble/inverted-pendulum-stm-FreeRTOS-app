/*   
 *  Filter coefficients uesd while testing pendulum are defined in this file
 *  to keep main_LIP.c reasonably clean
*/

#define FIR_1 \
    {\
        -0.030435920, -0.016287268, 0.0098560303, 0.044013896, 0.080986104, 0.11514397, \
        0.14128727, 0.15543592, 0.15543592, 0.14128727, 0.11514397, 0.080986104, \
        0.044013896, 0.0098560303, -0.016287268, -0.030435920 \
    };
