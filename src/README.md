# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Manual Tuning Process

It was very easy to implement a simple PID steering controller, so I had a lot of time initially to play with the gains. As I learned in the lessons, I found that the proportional gain improves response time, but a proportional gain too high causes oscillations and eventually instability. The differential gain reduces oscillations, and usually has to be bigger than P gain to reduce the error. But if D gain is too big, oscillations appear again, and the car moves around very nervously. Finally the I gain reduces steady state error. This was most apparent on a straight line. But a large I value causes instability. So usually the I value is magnitudes smaller than P. 

I have tuned the PID control manually. My method is based on George Gillard's method referenced in the forum. 

* I start with just tuning P, until the vehicle can manage all the corners with oscillation. 
* Then I tune D to reduce the oscillation. 
* After that, I add very small amount of I to fix and steady state errors for the straght line.

## Twiddle

Afterwards, I implemented the twiddle code from Sebastian's lecture. It was a bit tricky, considering that we can't simply do robot.run(), and have to get around the whole simulator code. So the twiddle code ends up being spread around in the PID.cpp and main.cpp. I'll describe the main idea here. 

* I start with the manually tuned PID values, and set the "dp" "step size to be 1/10th of the gain values 
* I run the simulator for 8500 steps until the vehicle crosses the bridge and goes through the three sharp turns, all the while accumulating the total cross track error squared. 
* After 8500 steps, the vehicle resets, and I run the twiddle logic. The idea is that if the parameter changed helped the performance, increase the parameter change, if not, decrease the change to get closer to the optimal value. 
* After 169 cycles, the "dp" all become smaller, and converges to a sum lower than 0.005, then I stop twiddle. 

With Twiddle, I ended up with P: 0.55, I: 0.0011, D: 10.793. A video is attached to show the performance.
