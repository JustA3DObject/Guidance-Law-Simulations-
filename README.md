# PLATO
PLATO is a new model for controlling speed profile of an AUV in it's "homing" state. Combined with different guidance laws to correct cross-track error and heading error, PLATO can bring an AUV to it's dock with a minimum required velocity countring the current disturbances and the speed of the dock it self. 
The repository contains two different MATLAB scipts to simualte AUV homing mission.
One contains PLATO combined with LOS, Carrot chase and Non Linear Guidance (NLG) 
Other contains log-polynomial law with a cubic polynomial, used for controlling speed profiles of rockets or other aerial vehicles in their homing phase for providing them a soft landing, combined with same three laws as above for comparison with PLATO.
The results are generated with two different heading angles, 45 and 225. There are initial heading angles of the AUV wrt to the dock at time = 0.
