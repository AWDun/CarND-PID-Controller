#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double K_p, double K_i, double K_d, bool twdl, int itr) {
	//initialize errors
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
	
	//initialize gain parameters given in main
	Kp = K_p;
	Ki = K_i;
	Kd = K_d;
	
	//initialze twiddle parameters if twiddle is activated
	twiddle = twdl;
	max_itr = itr;
	if(twiddle == true){
		p = {Kp+0.1*Kp, Ki, Kd};
		dp = {0.1*Kp, 0.1*Ki, 0.1*Kd}; //start with small steps around the value of the manual gains
		cte_total = 0.0;
		cte_best = 10000000000.0; //pick a large number
		n = 0;
		tot_cycles = 0;
		i_p = 0; 
		twid_state = 0;
	}
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error = i_error + cte;
	
	if(twiddle == true){
		cte_total += cte*cte; //take square of cte to keep all errors positive
		n += 1; //update simulator cycle count of a twiddle cycle
		cout << "n: " << n << " total cycles: " << tot_cycles << endl;
	}
}

double PID::TotalError() {
	
	//PID equation
	double steering = -Kp*p_error - Kd * d_error - Ki * i_error;
	
	//Saturation of steering
	if(steering >=1.0){
		steering = 1.0;
	}
	else if (steering <= -1.0){
		steering = -1.0;
	}
	
	if(twiddle == true) {
		
		//during twiddle, gain parameters need to be constantly updated
		Kp = p[0];
		Ki = p[1];
		Kd = p[2];
		steering = -Kp*p_error - Kd * d_error - Ki * i_error;
		
		//debug messages for twiddle
		cout << "P: " << Kp << " I: " << Ki << " D: " << Kd << endl;
		cout << "i_p: " << i_p << endl;
		cout << "cte_total: " << cte_total << " cte_best: " << cte_best << " twid_stat: " << twid_state << endl;
		cout << "dp: " << dp[0] << " " << dp[1] << " " << dp[2] << endl;
	}
	return steering;
}

bool PID::TwiddleCount() {
	bool reset = 0;
	
	//if simulator has been running for max_itr cycles, reset simulator
	if(n >= max_itr){
		reset = 1;
		n = 0;
		tot_cycles += 1;
		//reset errors when simulator resets
		p_error = 0.0;
		i_error = 0.0;
		d_error = 0.0;
		
		//twiddle logic below, modified to accomodate structure of this code
		if(cte_total < cte_best){
			cte_best = cte_total;
			dp[i_p] *= 1.1;
			i_p += 1;
			if(i_p > 2){
				i_p = 0;
			}
			p[i_p] += dp[i_p];
			twid_state = 0;
			cout << "Better!" << endl;
		}
		else if(twid_state == 0){
			p[i_p] -= 2*dp[i_p];
			twid_state = 1;
			cout << "Worse, try again!" << endl;
		}
		else if(twid_state == 1){
			p[i_p] += dp[i_p];
			dp[i_p] *= 0.9;
			i_p += 1;
			if(i_p > 2){
				i_p = 0;
			}
			twid_state = 0;
			p[i_p] += dp[i_p];
			cout << "Worse, move on" << endl;
		}
		//reset cte total after each cycle
		cte_total = 0.0;
	}
	return reset;
}

int PID::TwiddleDone() {
	bool done = 0;
	if (twiddle == true){
		if((dp[0]+dp[1]+dp[2]) < 0.005) //end condition for twiddle
			done = 1;
	}
	return done;
}
