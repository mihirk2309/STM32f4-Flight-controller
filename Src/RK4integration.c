#include "RK4integration.h"
/*
@Brief: The function is used to give the value of output variable y for value x of input variable
@param: x0 is the initial value of x for which the output is known
@param: y0 is the known output value for the known input x0
@param: x is the input value at which the output needs to be calculated
@param: h is the step size in input variable
@param: dxdy is a pointer to the function( i.e. first order diff. equation) defining relation between input and output
@retval: The output value at the input x
*/
float RungeKutta4(float x0,float y0,float x,float h,float (*dydx)(float,float))
{
	int n = (int)((x - x0) / h); 
  
    float k1, k2, k3, k4, k5; 
 
    // Iterate for number of iterations 
    float y = y0; 
    
        // Apply Runge Kutta Formulas to find 
        // next value of y 
        k1 = h*(*dydx)(x0, y); 
        k2 = h*(*dydx)(x0 + 0.5*h, y + 0.5*k1); 
        k3 = h*(*dydx)(x0 + 0.5*h, y + 0.5*k2); 
        k4 = h*(*dydx)(x0 + h, y + k3); 

        // Update next value of y 
        y = y + (1.0/6.0)*(k1 + 2*k2 + 2*k3 + k4);; 
  
        // Update next value of x 
        x0 = x0 + h ; 
     
    return y; 
} 
