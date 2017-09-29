#ifndef PID_H
#define PID_H
#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
	
	/*
	 * Twiddle error memory
	*/
	double cte_total;
	double cte_best;
	
	/*
	 * Twiddle cycles
	 */
	int n; // current iteration number of a twiddle cycle
	int tot_cycles; // number of twiddle cycles
	int max_itr; // max number of iterations per cycle
	int i_p; //current parameter being tested
	
	/*
	 * Twiddle Parameters
	 */
	bool twiddle;
	std::vector<double> p;
	std::vector<double> dp;
	int twid_state; //keeps track of where in the "if-else" cycle twiddle is at
	
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, bool twiddle, int max_itr);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
	
	/*
	* Function to check whether to reset simulator
	*/
	bool TwiddleCount();
	
	/*
	 * Check if twiddle is done
	 */
	int TwiddleDone();
};

#endif /* PID_H */
