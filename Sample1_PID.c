struct PID_parameters {
	const float Kp;
	const float Ki;
	const float Kd;
	const float low_limit;
	const float upper_limit;
	const float Samp_time;
	const float s;
	float proportional;
	float integrator;
	float differentiator;
	const unsigned char Ftr_coef;
};

struct  PID_parameters parameters= {
		.Kp = 0.6696,
		.Ki = 13.392,
		.Kd = 0.00837,
		.Ftr_coef = 10,
		.Samp_time = 0.02
};

float DAC_Vlt_conversion(float dac)
{
	return 0.0014 * dac;
}

float Vlt_DAC_conversion(float vlt)
{
	return 714.2857 * vlt;
}

float PID_controller(struct PID_parameters *pid, float set, float msr)
{
	static float prv_msr, prv_err;
	float error;
	
	set = 198.56 * (pow(set,0.52));
	msr = 198.56 * (pow(msr,0.52));

	set = DAC_Vlt_conversion(set);
	msr = DAC_Vlt_conversion(msr);
	error = set - msr;

	/*To calculate proportional, integrator, differentiator */
	pid->proportional = pid->Kp * error;
	pid->integrator = (pid->Ki * (msr - prv_msr)) * 1 / pid->s;
	pid->differentiator = -pid->Kd * (error - prv_err)/pid->Samp_time * (pid->Ftr_coef/(1 + pid->Ftr_coef/pid->s));

	prv_msr = msr;
	prv_err = error;

	return Vlt_DAC_conversion(pid->proportional + pid->integrator + pid->differentiator);
}