Complete autotuning with out stored array of data

 - important variables:
        ykm1    previous output
        yk      current output
        y0      initial output
        yf      final or steady state step-response
        ysmax   y at point where smax is acheived
        tsmax   time where maximum output is found
        tss     time where the process attain steady-state value
 - challenge:
        how to determine the half-energy time and output without looking back or registering in array
        
        # memory vs time cost
        # for the instant position, memory taken
