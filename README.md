# Pid-controller-Algorithim

This library is under the MIT LICENSE such that the  Commercial use, Modification, Distribution, and Private use is allowed
with no Warranty and Liability. 

The PID controller algorithim is developed in C-programing language to ease its implementation in embedded system. Therefore, the algorithm can be imported to any embedded system project whose IDE supports C-programming (C-99). The library provides the discrete PID controller with Derivative filter and Antiwind-up scheme which is tuned using Matlab simulink. The simulink provides the choice for the integral and the derivative filter type where by default both are set to Forward Euler (S to Z mapping). Besides, the real world application of the controllers has a limit in magnitude of input and output signal. For instance, if the setpoint is read using ADC converter of micro-controller, the maximum input value might be 1023 or 255 (3.3 Volt or 5.0 Volts). In practical application of controller, the PWM is used to generate control signal, but this signal is not strong enough to drive plants (eg. motor). Therefore, amplifier or driver with limited capacity is used to generate the control signal which is strong enough to drive plant. The driver constraints also limits the control effort that might be very large, consequently, the output actuator limit is soft coded in the progam.

For further advancement of the project, participation is highly appreciated in:
    1. Developing auto-tune function
    2. Converting the algorithm to C++ to support Arduino IDE
    3. Testing and compiling the library in any micro-controller
