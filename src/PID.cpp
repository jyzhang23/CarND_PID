#include "PID.h"

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;
	
	p_error = 0;
	i_error = 0;
	d_error = 0;
	
	//twiddle parameters
	dp = Kp * 0.1; // change parameters by 10%
	di = Ki * 0.1;
	dd = Kd * 0.1;
	
	tol = .01;
	best_err = 9999;
	window_size = 10; //window size of last x values of cte
	n=1;
}

void PID::UpdateError(double cte) {
	if (n == 1){
		p_error = cte; //on first pass, this would be zero
	}
	d_error = cte - p_error; //assume dt is same for all instances
	p_error = cte;
	if (cte_window.size() >= window_size){
		cte_window.pop_front();
	}
	cte_window.push_back(cte);
	if (n>20){
		i_error = accumulate(begin(cte_window), end(cte_window), 0.0);
	}
	n++;
}

double PID::TotalError() {
	return p_error + i_error + d_error;
}

void PID::Twiddle(double cte){
	double tot_err = fabs(TotalError());
	if(n > 100 && tot_err < tol){
		cout<<"Success! Final parameters - Kp: "<<Kp<<", Ki: "<<Ki<<", Kd: "<<Kd<<endl;
		return;
	} else {
		cout<<"twiddling: total error = "<<tot_err<<", best error = "<<best_err<<endl;
		if(n > 75){
			Kp += dp;
			Ki += di;
			Kd += dd;
			if(tot_err < best_err){
				best_err = tot_err;
				dp *= 1.05;
				di *= 1.05;
				dd *= 1.05;
				cout<<"Improvement! Increasing change rate by 10%"<<endl;
				cout<<"Kp: "<<Kp<<", Ki: "<<Ki<<", Kd: "<<Kd<<endl;
				cout<<"dp: "<<dp<<", di: "<<di<<", dd: "<<dd<<endl;
			} else {
				Kp -= 2 * dp;
				Ki -= 2 * di;
				Kd -= 2 * dd;
				cout<<"Got worse! Trying negative..."<<endl;
				cout<<"Kp: "<<Kp<<", Ki: "<<Ki<<", Kd: "<<Kd<<endl;
				cout<<"dp: "<<dp<<", di: "<<di<<", dd: "<<dd<<endl;
				if(tot_err < best_err){
					best_err = tot_err;
					dp *= 1.05;
					di *= 1.05;
					dd *= 1.05;
					cout<<"Improvement! Increasing change rate by 10%"<<endl;
					cout<<"Kp: "<<Kp<<", Ki: "<<Ki<<", Kd: "<<Kd<<endl;
					cout<<"dp: "<<dp<<", di: "<<di<<", dd: "<<dd<<endl;
				} else {
					Kp += dp;
					Ki += di;
					Kd += dd;
			
					dp *= .95;
					di *= .95;
					dd *= .95;
					cout<<"Got worse! Decreasing change rate by 10%"<<endl;
					cout<<"Kp: "<<Kp<<", Ki: "<<Ki<<", Kd: "<<Kd<<endl;
					cout<<"dp: "<<dp<<", di: "<<di<<", dd: "<<dd<<endl;
				}
			}
		}
	UpdateError(cte);
	}
}
