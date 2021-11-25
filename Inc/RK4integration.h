
/*
@Brief: The function is used to give the value of output variable y for value x of input variable
@param: x0 is the initial value of x for which the output is known
@param: y0 is the known output value for the known input x0
@param: x is the input value at which the output needs to be calculated
@param: h is the step size in input variable
@param: dxdy is a pointer to the function( i.e. first order diff. equation) defining relation between input and output
@retval: The output value at the input x
*/
float RungeKutta4(float x0,float y0,float x,float h,float (*dydx)(float,float));
