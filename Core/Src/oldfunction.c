// motor setting
// GM2804 motor
//uint16_t AS5048_zeropos = 1945;
//#define polepair 7
//#define P 0.8f
//#define I (float)1650/20000
//#define Ka 1.25f

// big motor
//uint16_t AS5048_zeropos = 5739;
//#define polepair 15
//#define P 0.3f
//#define I (float)210/20000
//#define Ka (float)10/P

/* TIM1 ISR start */
/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){  //TIM interrupt rank has to be added in main()
	if(htim->Instance == TIM1){
		AS5048A_ReadAngle();  // read AS5048A angle value and store at mech_angle

		if(j == 0){
			mech_speed = 0.0;
			j++;
		}
		else if(j++ == 11){
			int speed_delta = mech_angle - mech_angle_prev;
			if(speed_delta > 8192){
				speed_delta -= 16384;
			}
			else if(speed_delta < -8192){
				speed_delta += 16384;
			}

			mech_speed = (float) (speed_delta / 16384.0f * 6.28318530f * 2000.0f);

			mech_angle_prev = mech_angle;

			j = 1;
		}

		float delta = (float) (mech_angle - AS5048_zeropos);
		if(delta < 0.0) delta = delta + 16384.0;

		while(delta > ELEC_CNT){
			delta = delta - ELEC_CNT;
		}

		electrical_angle = delta * 360.0f / ELEC_CNT;

		if (electrical_angle > 180.0f) electrical_angle -= 360.0f;

		arm_sin_cos_f32(electrical_angle,&_sin,&_cos);

		ic = current_A[0] - current_A_calibrated;  // A<->C mapping hardware issue
		ib = current_B[0] - current_B_calibrated;
		//ia = current_C[0] - current_C_calibrated;  // A<->C mapping hardware issue
		ia = -(ib+ic); // ia is very unstable. currnet_C has big jitter (maybe hardware problem)

		// inv clarke
		//0.01 ohm(shunt) * i * 10V/V(Op-amp gain) = (x - x_cali) / 4096 * 3.3 -> i = (x - x_cali) * 33 / 4096 = (x - x_cali) * 0.00805664
		//0.005 ohm(shunt) -> i = (x - x_cali) * 0.01611328 (minimum sensing current 16mA)
		i_alpha =  ia * 0.01611328;
		i_beta  = (ib - ic) * invsqrt3 * 0.01611328;

		// current LPF
		i_alpha = (1 - ALPHA) * i_alpha_prev + ALPHA * i_alpha;
		i_beta  = (1 - ALPHA) * i_beta_prev  + ALPHA * i_beta;
		i_alpha_prev = i_alpha;
		i_beta_prev  = i_beta;

		// inv park
		id =  i_alpha * _cos + i_beta * _sin;
		iq = -i_alpha * _sin + i_beta * _cos;


		//PI
		iq_err = iq_ref - iq;
		id_err = id_ref - id;
		iq_err_integral = iq_err_integral + iq_err;
		id_err_integral = id_err_integral + id_err;

//		float Rs = 4.5;
//		float Ls = 0.8*0.001;
//		float phi = 5.503*0.001;
//		float we = mech_speed * polepair;
//		// Vq, Vd calculation
//		Vq_unsat = we*(Ls*id+phi) + (P * iq_err) + (I * iq_err_integral);  //V = V/A * A + V/(A*Ts) * A*Ts
//		Vd_unsat = -we*(Ls*iq) + (P * id_err) + (I * id_err_integral);
//
//		float Rs = 0.4585;
//		float Ls = 0.8*0.001;
//		float phi = 0.025644;
//
//		we = mech_speed * polepair;
//
//		we  = (1 - we_ALPHA) * i_beta_prev  + we_ALPHA * we;
//		we_prev = we;
//
//		// Vq, Vd calculation
//		Vq_unsat = we*(Ls*id+phi) + (P * iq_err) + (I * iq_err_integral);  //V = V/A * A + V/(A*Ts) * A*Ts
//		Vd_unsat = - we*(Ls*iq)   + (P * id_err) + (I * id_err_integral);

		// Vq, Vd calculation
		Vq_unsat = (P * iq_err) + (I * iq_err_integral);
		Vd_unsat = (P * id_err) + (I * id_err_integral);

		Vd = (Vd_unsat > Vd_limit) ? Vd_limit : (Vd_unsat < -Vd_limit) ? -Vd_limit : Vd_unsat;

		Vq_limit = sqrt((V_LIMIT*V_LIMIT) - (Vd*Vd));
		Vq = (Vq_unsat > Vq_limit) ? Vq_limit : (Vq_unsat < -Vq_limit) ? -Vq_limit : Vq_unsat;

		iq_err_integral = iq_err_integral - Ka*(Vq_unsat - Vq);
		id_err_integral = id_err_integral - Ka*(Vd_unsat - Vd);

		// park
		V_alpha = Vd * _cos - Vq * _sin;
		V_beta  = Vd * _sin + Vq * _cos;

		if(enable_motor){
			SVPWM(-V_alpha, -V_beta);

			if(i++ == 2000){
				i = 0;
				iq_ref = -iq_ref;
			}
		}
		else if(encoder_cali){
			V_arg = V_arg + V_cali_speed;
			SVPWM(V_mag * cos(V_arg), V_mag * sin(V_arg));
		}
		else
		{
			SVPWM(0.0,0.0);
			iq_err_integral = 0.0;
			id_err_integral = 0.0;
		}

	  }
	  return;
}*/

/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){  //TIM interrupt rank has to be added in main()
	if(htim->Instance == TIM1){
		AS5048A_ReadAngle(); // read AS5048A angle value and store at mech_angle
		float delta = (float) (mech_angle - AS5048_zeropos);
		if(delta < 0.0)
			delta = delta + 16384.0;

		while(delta > ELEC_CNT){
			delta = delta - ELEC_CNT;
		}
		electrical_angle = delta * 360.0f / ELEC_CNT;

		if (electrical_angle > 180.0f)
			electrical_angle -= 360.0f;

		V_arg = V_arg + V_cali_speed;
		if(V_arg > 6.28318530718)
			V_arg -= 6.28318530718;

		ic = current_A[0] - current_A_calibrated;  // A<->C mapping hardware issue
		ib = current_B[0] - current_B_calibrated;
		//ia = current_C[0] - current_C_calibrated;  // A<->C mapping hardware issue
		ia = -(ib+ic); // ia is very unstable. currnet_C has big jitter (+-10 LSB) (maybe hardware PCB problem. maybe charge pump?)

		// inv clarke
		//0.01 ohm(shunt) * i * 10V/V(Op-amp gain) = (x - x_cali) / 4096 * 3.3 -> i = (x - x_cali) * 33 / 4096 = (x - x_cali) * 0.00805664
		//0.005 ohm(shunt) * i * 20V/V(Op-amp gain) = (x - x_cali) / 4096 * 3.3 -> i = (x - x_cali) * 0.00805664

		#define current_const 0.00805664*2

		i_alpha = ia * current_const;                   // i_alpha = 2/3 * (ia - (ib + ic)/2) = ia
		i_beta  = (ib - ic) * invsqrt3 * current_const; // i_beta  = 2/3 * (ib - ic) * sqrt(3)/2 = (ib - ic) * 1/sqrt(3)
		// 25.11.03 : measured real current with oscilloscope current probe, +-2 LSB ADC error. current is correct.


		// current LPF
		//i_alpha = (1 - ALPHA) * i_alpha_prev + ALPHA * i_alpha;
		//i_beta  = (1 - ALPHA) * i_beta_prev  + ALPHA * i_beta;
		//i_alpha_prev = i_alpha;
		//i_beta_prev  = i_beta;

		// inv park
		id =  i_alpha * _cos + i_beta * _sin;
		iq = -i_alpha * _sin + i_beta * _cos;

		if(HFI_test){
			if(flag){
				SVPWM(2.0, 0.0) // Alien to alpha axis

			}
			V_alpha = V_mag * cos(V_arg) + Vd_injected;
			V_beta = V_mag * sin(V_arg) + Vd_injected;

			SVPWM(V_alpha, V_beta);

			if(i++ == 4){
				i = 0;
				Vd_injected = -Vd_injected;
			}
		}
		else if(measure_R){
			if(j < 5000){
				if(i++ == 2){
					i = 0;

					j++;

					V_alpha = 0.001 * j;
					log_1[j] = V_alpha;
				}
				if(i == 1){
					log_2[j] = i_alpha;
				}
				SVPWM(V_alpha, V_beta);
			}
			else{
				V_alpha = 0.0;
				SVPWM(0.0, 0.0);
			}
		}
		else if(motor_copper_loss){
			if(j < 10000){
				log_1[j] = V_alpha;
				log_2[j] = i_alpha;

				const float T_chirp   = 0.5f;        // chirp 전체 시간 0.5 s
				const float f0        = 0.0f;        // 시작 주파수 [Hz]
				const float f1        = 1000.0f;     // 끝 주파수 [Hz]
				const float k         = (f1 - f0) / T_chirp;  // 주파수 기울기 [Hz/s]

		        float t     = j * 50 * 1e-6;   // 현재 시간 [s]
		        float phase = 2.0f * 3.14159265359f * (f0 * t + 0.5f * k * t * t);

				V_alpha = V_mag * arm_sin_f32(phase);

				SVPWM(V_alpha, 0.0);

				j++;
			}
			else{
				motor_copper_loss = 0;
				SVPWM(0.0,0.0);
			}
		}
		else
		{
			SVPWM(0.0,0.0);
		}

		if(log_on){
			if(j < 5000){
				log_1[j] = electrical_angle;
				log_2[j] = i_alpha;
				log_3[j] = i_beta;
				log_4[j] = V_alpha;
				log_5[j] = V_beta;

				j++;
			}
		}
	}
	return;
}*/

/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){  //TIM interrupt rank has to be added in main()
	if(htim->Instance == TIM1){
//		AS5048A_ReadAngle(); // read AS5048A angle value and store at mech_angle
//		float delta = (float) -(mech_angle - AS5048_zeropos);
//		if(delta < 0.0)
//			delta = delta + 16384.0;
//
//		while(delta > ELEC_CNT){
//			delta = delta - ELEC_CNT;
//		}
//		electrical_angle = delta * 360.0f / ELEC_CNT;
//
//		if (electrical_angle > 180.0f)
//			electrical_angle -= 360.0f;

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1);
//		for(uint16_t wait=0; wait<100; wait++) __NOP();

		// alpha beta current calculation
		ic = current_A[0] - current_A_calibrated;  // A<->C mapping hardware issue
		ib = current_B[0] - current_B_calibrated;
		//ia = current_C[0] - current_C_calibrated;  // A<->C mapping hardware issue
		ia = -(ib+ic); // ia is very unstable. currnet_C has big jitter (+-10 LSB) (maybe hardware PCB problem. maybe charge pump?)

		// inv clarke
		//0.005 ohm(shunt) * i * 20V/V(Op-amp gain) = (x - x_cali) / 4096 * 3.3 -> i = (x - x_cali) * 0.00805664
		#define current_const 0.00805664*2  // 10V/V Op-amp gain

		i_alpha = ia * current_const;                   // i_alpha = 2/3 * (ia - (ib + ic)/2) = ia
		i_beta  = (ib - ic) * invsqrt3 * current_const; // i_beta  = 2/3 * (ib - ic) * sqrt(3)/2 = (ib - ic) * 1/sqrt(3)
		// 25.11.03 : measured real current with oscilloscope current probe, +-2 LSB ADC error. current is correct.

		// digital LPF or sum
		//i_alpha_LPF = (1 - deadtime_ALPHA) * i_alpha_LPF + deadtime_ALPHA * i_alpha;
		//i_beta_LPF  = (1 - deadtime_ALPHA) * i_beta_LPF  + deadtime_ALPHA * i_beta;
		i_alpha_LPF = (i_alpha + i_alpha_prev) / 2;
		i_beta_LPF  = (i_beta  + i_beta_prev)  / 2;

		i_alpha_prev = i_alpha;
		i_beta_prev  = i_beta;

		// peak current cutoff
		if( (i_alpha > peak_current_limit) | (i_alpha < -peak_current_limit) | (i_beta > peak_current_limit) | (i_beta < -peak_current_limit)){
			HFI_test = 0;
			if(OC_flag){
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
				OC_flag = 0;
			}

		}
		// HFI start
		if(HFI_test){
			if(flag){
				SVPWM(2.0, 0.0); // alpha axis align, delete later
				estimated_electrical_angle = 0.0;

				// wait 4 sec
				if(count++ == 39999){
					flag = 0;
					count = 0;
				}
			}
			else{
				// rotor angle estimation with HFI
				if(i == 1){  // one tick after V_injected flip
					if(have_prev_HFI){
						delta_i_alpha = i_alpha - i_alpha_prev_HFI;
						delta_i_beta  = i_beta  - i_beta_prev_HFI;

						if(V_injected > 0){
							delta_i_alpha = - delta_i_alpha;
							delta_i_beta  = - delta_i_beta;
						}

						theta_e_est_rad = atan2f(delta_i_beta, delta_i_alpha);

						if(have_prev_estimation){
							float case1 = theta_e_est_rad;
							float case2 = theta_e_est_rad + PI;

							if(fabsf(wrap_pm_pi(case1 - theta_e_est_rad_prev)) > fabsf(wrap_pm_pi(case2 - theta_e_est_rad_prev))){
								theta_e_est_rad = case2;
								angle_flip = 1;
							}
							else{
								theta_e_est_rad = case1;
								angle_flip = 0;
							}
							theta_e_est_rad_prev = theta_e_est_rad;
						}
						else{
							theta_e_est_rad_prev = theta_e_est_rad;
						}
						have_prev_estimation = 1;

//						float choose = theta_e_est_rad;
//						if(choose < 0)
//							choose = choose + 2*PI;
//
//						if(abs(k - (uint8_t)(choose / SECTOR_RAD) > 2))
//							theta_e_est_rad = wrap_pm_pi(theta_e_est_rad + PI);

						estimated_electrical_angle = theta_e_est_rad * (180.0f / PI);

						//printf("%.3f,%.3f,%.3f,%.3f,%d\n", estimated_electrical_angle, delta_i_alpha, delta_i_beta, V_injected, angle_flip);
					}

					i_alpha_prev_HFI = i_alpha;
					i_beta_prev_HFI  = i_beta;
					have_prev_HFI    = 1;

					// if HFI injection angle is shifted 180deg, compensate (not using now)
//				   if((theta_e_est_rad_prev - theta_e_est_rad < -PI/2) | (theta_e_est_rad_prev - theta_e_est_rad > PI/2)){
//						//V_injected = -V_injected;
//						j++;
//					}
//
//					//theta_e_est_rad_prev = theta_e_est_rad;
//
//					//printf("%.3f,%.3f\n", electrical_angle, estimated_electrical_angle);
//					//if(log_jump++ == 4){
//						//printf("%d,%.3f,%.3f,%.3f\n", mech_angle, V_arg, electrical_angle, estimated_electrical_angle);
//
//					//	log_jump = 0;
//					//}
//
//					if(log_on){]
//						if(j < 5000){
//							log_1[j] = electrical_angle;
//							log_2[j] = estimated_electrical_angle;
//							log_3[j] = delta_i_alpha;
//							log_4[j] = delta_i_beta;
//							log_5[j] = 0.0;
//
//							j++;
//						}
//					}
				}

				if(count < 19999){ //estimate start, wait 1 sec to estimation converge
					V_alpha = V_injected * cos(theta_e_est_rad);
					V_beta =  V_injected  * sin(theta_e_est_rad);
					count++;
				}
				else{
					arm_sin_cos_f32(estimated_electrical_angle,&_sin,&_cos);

					// inv park, using high frequency rejected
					id =  i_alpha_LPF * _cos + i_beta_LPF * _sin;
					iq = -i_alpha_LPF * _sin + i_beta_LPF * _cos;

					//PI
					iq_err = iq_ref - iq;
					id_err = id_ref - id;
					iq_err_integral = iq_err_integral + iq_err;
					id_err_integral = id_err_integral + id_err;

					// Vq, Vd calculation
					Vq_unsat = (P * iq_err) + (I * iq_err_integral);
					Vd_unsat = (P * id_err) + (I * id_err_integral);

					Vd = (Vd_unsat > Vd_limit) ? Vd_limit : (Vd_unsat < -Vd_limit) ? -Vd_limit : Vd_unsat;

					Vq_limit = sqrt((V_LIMIT*V_LIMIT) - (Vd*Vd)) * 0.9; // 0.9 for leave injection voltage
					Vq = (Vq_unsat > Vq_limit) ? Vq_limit : (Vq_unsat < -Vq_limit) ? -Vq_limit : Vq_unsat;

					iq_err_integral = iq_err_integral - Ka*(Vq_unsat - Vq);
					id_err_integral = id_err_integral - Ka*(Vd_unsat - Vd);

					// park
					V_alpha = Vd * _cos - Vq * _sin + V_injected * cos(theta_e_est_rad); // d axis injection
					V_beta  = Vd * _sin + Vq * _cos + V_injected * sin(theta_e_est_rad); // d axis injection

					//printf("%.3f,%.3f,%.3f,%.3f\n", iq, id, iq_ref, estimated_electrical_angle);

					if(log_jump++ == 49999){
						//iq_ref = -iq_ref;
						log_jump = 0;
					}
				}

				logging[1] = delta_i_alpha;
				logging[2] = delta_i_beta;
				logging[3] = estimated_electrical_angle;
				logging[4] = (float) angle_flip;
				logging[5] = V_injected;

				SVPWM(V_alpha, V_beta);

				if(i++ == 1){  // injection flip with 5 tick -> 20 kHz / (2 * 5tick) = 2kHz injection square wave voltage (i = 4) //now 10k (i = 1)
					i = 0;
					V_injected = -V_injected;
				}

				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
			}
		}
//		else if(encoder_cali){
//			V_arg = V_arg + V_cali_speed;
//			if(log_jump++ == 9){
//				printf("%.3f\n", electrical_angle);
//				log_jump = 0;
//			}
//			if(V_arg > 2*PI)
//				V_arg -= 2*PI;
//			SVPWM(V_mag * cos(V_arg), V_mag * sin(V_arg));
//		}

//		else if(measure_R){
//			if(j < 5000){
//				if(i++ == 4){
//					i = 0;
//
//					j++;
//
//					V_alpha = 0.001 * j;
//					log_1[j] = V_alpha;
//				}
//				if(i == 1){
//					log_2[j] = i_alpha;
//				}
//				SVPWM(V_alpha, V_beta);
//			}
//			else{
//				V_alpha = 0.0;
//				SVPWM(0.0, 0.0);
//			}
//		}
//		else if(SV_test){
//			V_arg = V_arg + V_cali_speed;
//			if(V_arg > 6.28318530718)
//				V_arg -= 6.28318530718;
//
//			V_alpha = V_mag * cos(V_arg);
//			V_beta =  V_mag * sin(V_arg);
//
//			if(log_jump++ == 9){
//				printf("%.3f,%.3f,%.3f,%.3f,%d,%d,%d\n", V_alpha_prev_, i_alpha, V_beta_prev_, i_beta, t0, t1, t2);
//				log_jump = 0;
//			}
//
//			V_alpha_prev_ = V_alpha;
//			V_beta_prev_  = V_beta;
//
//			SVPWM(V_alpha, V_beta);
//		}
		else{
			SVPWM(0.0,0.0);
		}
	}
	return;
}*/

/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){  //TIM interrupt rank has to be added in main()
	if(htim->Instance == TIM1){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1);
		uint32_t debug_time_0 = dwt_now();

//		for(uint16_t wait=0; wait<100; wait++) __NOP();

		// alpha beta current calculation
		ic = current_A[0] - current_A_calibrated;  // A<->C mapping hardware issue
		ib = current_B[0] - current_B_calibrated;
		//ia = current_C[0] - current_C_calibrated;  // A<->C mapping hardware issue
		ia = -(ib+ic); // ia is very unstable. currnet_C has big jitter (+-10 LSB) (maybe hardware PCB problem. maybe charge pump?)

		// inv clarke
		//0.005 ohm(shunt) * i * 20V/V(Op-amp gain) = (x - x_cali) / 4096 * 3.3 -> i = (x - x_cali) * 0.00805664
		#define current_const 0.00805664*2  // 10V/V Op-amp gain

		i_alpha = ia * current_const;                   // i_alpha = 2/3 * (ia - (ib + ic)/2) = ia
		i_beta  = (ib - ic) * invsqrt3 * current_const; // i_beta  = 2/3 * (ib - ic) * sqrt(3)/2 = (ib - ic) * 1/sqrt(3)
		// 25.11.03 : measured real current with oscilloscope current probe, +-2 LSB ADC error. current is correct.

		// digital LPF or sum
		i_alpha_LPF = (i_alpha + i_alpha_prev) / 2;
		i_beta_LPF  = (i_beta  + i_beta_prev)  / 2;

		i_alpha_prev = i_alpha;
		i_beta_prev  = i_beta;

		// peak current cutoff
		if( (i_alpha > peak_current_limit) | (i_alpha < -peak_current_limit) | (i_beta > peak_current_limit) | (i_beta < -peak_current_limit)){
			HFI_test = 0;
			if(OC_flag){
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
				OC_flag = 0;
			}
		}

		uint32_t debug_time_1 = dwt_now();

		// HFI start
		if(HFI_test){
			if(flag){
				SVPWM(2.0, 0.0); // alpha axis align, delete later
				estimated_electrical_angle = 0.0;

				// wait 4 sec
				if(count++ == 39999){
					flag = 0;
					count = 0;
				}
			}
			else{
				// rotor angle estimation with HFI
				if(i == 1){  // one tick after V_injected flip
					if(have_prev_HFI){
						delta_i_alpha = i_alpha - i_alpha_prev_HFI;
						delta_i_beta  = i_beta  - i_beta_prev_HFI;

						if(V_injected > 0){
							delta_i_alpha = - delta_i_alpha;
							delta_i_beta  = - delta_i_beta;
						}

						theta_e_est_rad = atan2f(delta_i_beta, delta_i_alpha);

						if(have_prev_estimation){
							float case1 = theta_e_est_rad;
							float case2 = theta_e_est_rad + PI;

							if(fabsf(wrap_pm_pi(case1 - theta_e_est_rad_prev)) > fabsf(wrap_pm_pi(case2 - theta_e_est_rad_prev))){
								theta_e_est_rad = case2;
								angle_flip = 1;
							}
							else{
								theta_e_est_rad = case1;
								angle_flip = 0;
							}
							theta_e_est_rad_prev = theta_e_est_rad;
						}
						else{
							theta_e_est_rad_prev = theta_e_est_rad;
						}
						have_prev_estimation = 1;

						estimated_electrical_angle = theta_e_est_rad * (180.0f / PI);
					}

					i_alpha_prev_HFI = i_alpha;
					i_beta_prev_HFI  = i_beta;
					have_prev_HFI    = 1;
				}

				uint32_t debug_time_2 = dwt_now();

				if(count < 19999){ //estimate start, wait 1 sec to estimation converge
					V_alpha = V_injected * _cos;
					V_beta =  V_injected * _sin;
					count++;
				}
				else{
					arm_sin_cos_f32(estimated_electrical_angle,&_sin,&_cos);

					// inv park, using high frequency rejected
					id =  i_alpha_LPF * _cos + i_beta_LPF * _sin;
					iq = -i_alpha_LPF * _sin + i_beta_LPF * _cos;

					//PI
					iq_err = iq_ref - iq;
					id_err = id_ref - id;
					iq_err_integral = iq_err_integral + iq_err;
					id_err_integral = id_err_integral + id_err;

					// Vq, Vd calculation
					Vq_unsat = (P * iq_err) + (I * iq_err_integral);
					Vd_unsat = (P * id_err) + (I * id_err_integral);

					Vd = (Vd_unsat > Vd_limit) ? Vd_limit : (Vd_unsat < -Vd_limit) ? -Vd_limit : Vd_unsat;

					Vq_limit = sqrt((V_LIMIT*V_LIMIT) - (Vd*Vd)) * 0.9; // 0.9 for leave injection voltage
					Vq = (Vq_unsat > Vq_limit) ? Vq_limit : (Vq_unsat < -Vq_limit) ? -Vq_limit : Vq_unsat;

					iq_err_integral = iq_err_integral - Ka*(Vq_unsat - Vq);
					id_err_integral = id_err_integral - Ka*(Vd_unsat - Vd);

					// park
					V_alpha = Vd * _cos - Vq * _sin + V_injected * _cos;//cos(theta_e_est_rad); // d axis injection
					V_beta  = Vd * _sin + Vq * _cos + V_injected * _sin;//sin(theta_e_est_rad); // d axis injection

					if(slow){
						slow_cos = cos(theta_e_est_rad);
						slow_sin = sin(theta_e_est_rad);
					}

					if(log_jump++ == 49999){
						//iq_ref = -iq_ref;
						log_jump = 0;
					}
				}

				uint32_t debug_time_3 = dwt_now();

				SVPWM(V_alpha, V_beta);

				if(i++ == 1){  // injection flip with 5 tick -> 20 kHz / (2 * 5tick) = 2kHz injection square wave voltage (i = 4) //now 10k (i = 1)
					i = 0;
					V_injected = -V_injected;
				}

				uint32_t debug_time_4 = dwt_now();

				current_processing_time = cycles_to_us(debug_time_1 - debug_time_0);
				HFI_time = cycles_to_us(debug_time_2 - debug_time_1);
				PI_time = cycles_to_us(debug_time_3 - debug_time_2);
				SVPWM_time = cycles_to_us(debug_time_4 - debug_time_3);

				isr_us = cycles_to_us(debug_time_4 - debug_time_0);

				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
			}
		}
		else{
			SVPWM(0.0,0.0);
		}
	}
	return;
}
*/

/*
static inline void PositionPD_Update_FromHFI(void){
    // HFI 각이 아직 안정화 전이면 제어하지 않게(필요시 조건 강화)
    if(count < 19999){
        return;
    }

    // 32-bit aligned float read는 Cortex-M에서 보통 원자적으로 읽힘(그래도 local copy 권장)
    float th_e = theta_e_wrapped_rad;

    static float th_e_prev = 0.0f;
    static float dth_e_acc = 0.0f;
    static float w_m_hold_prev = 0.0f;
	static uint8_t decim = 0;

    // 1) unwrap increment
    float dth_e = wrap_pm_pi(th_e - th_e_prev);
    th_e_prev = th_e;

    // 2) accumulate to continuous electrical angle
    float th_e_cont = theta_e_cont_rad + dth_e;
    theta_e_cont_rad = th_e_cont;

    // 3) mechanical position & speed
    float th_m = th_e_cont / polepair;
    pos_mech_rad = th_m;

    dth_e_acc += dth_e;

    if(++decim >= WM_DECIM){
    	float Ts_eff = POS_DT * (float)WM_DECIM;
    	w_m_hold = (dth_e_acc / Ts_eff) / polepair; // mech rad/s

    	w_m_hold = w_m_alpha * w_m_hold + (1 - w_m_alpha) * w_m_hold_prev;

    	w_m_hold_prev = w_m_hold;

    	dth_e_acc = 0.0f;
    	decim = 0;
    }

    // 4) position error (continuous)
    float e = pos_ref_mech_rad - th_m;

    // 5) PD (derivative on measurement to avoid derivative kick)
    P_part = pos_P * e;
    D_part = -(pos_D * w_m_hold);
    float iq_cmd = flip_sign * (P_part + D_part);

    // 6) saturate
    iq_cmd = (iq_cmd > IQ_REF_LIMIT) ? IQ_REF_LIMIT : (iq_cmd < -IQ_REF_LIMIT) ? -IQ_REF_LIMIT : iq_cmd;

    iq_ref = iq_cmd;
}
*/

/*
static inline void PositionPD_Update_FromHFI(void){
    // HFI 각이 아직 안정화 전이면 제어하지 않게(필요시 조건 강화)
    if(count < 19999){
        return;
    }

    // measured electrical angle from HFI (wrapped)
    float th_e_meas = theta_e_wrapped_rad;

    // =======================
    // (A) PI-PLL state (electrical)
    // =======================
    static uint8_t pll_inited = 0;

    static float th_e_hat = 0.0f;      // wrapped estimate
    static float we_hat   = 0.0f;      // electrical rad/s
    static float integ_e  = 0.0f;      // integral of phase error (rad*s)

    // for continuous angle build
    static float th_e_hat_prev = 0.0f;

    // for "new measurement" detection (HFI updates at 5kHz, TIM1 may be faster)
    static float th_e_meas_prev = 0.0f;

    // mech speed LPF (optional keep)
    static float w_m_hold_prev = 0.0f;

    if(!pll_inited){
        th_e_hat = th_e_meas;
        th_e_hat_prev = th_e_hat;
        we_hat = 0.0f;
        integ_e = 0.0f;

        // 기존 연속각 변수도 측정값으로 초기화(원하는 정책에 맞게 조정 가능)
        theta_e_cont_rad = th_e_meas;
        pos_mech_rad = theta_e_cont_rad / polepair;
        w_m_hold = 0.0f;

        th_e_meas_prev = th_e_meas;
        pll_inited = 1;
        return;
    }

    // HFI 각도 갱신이 5kHz라면, TIM1이 더 빠를 경우 같은 값이 여러번 들어올 수 있음
    // -> 갱신되지 않은 샘플에서는 PLL 업데이트를 생략(루프 이득이 Ts에 맞춰져 있으므로 중요)
    float dmeas = wrap_pm_pi(th_e_meas - th_e_meas_prev);
    if(fabsf(dmeas) < 1e-7f){
        // 측정 업데이트 없음: 이전 상태로 position/PD만 계산할지,
        // 아예 return할지 선택. 여기선 "그대로 계산"하도록 진행.
    } else {
        th_e_meas_prev = th_e_meas;

        // 1) phase error
        float e = wrap_pm_pi(th_e_meas - th_e_hat);

        // 2) integral
        integ_e += e * PLL_TS;

        // (optional) anti-windup: integral clamp so that L2*integ_e won't exceed speed limit too hard
        //float integ_lim = (PLL_WE_LIMIT / (PLL_L2 + 1e-12f));
        //integ_e = clampf(integ_e, -integ_lim, +integ_lim);

        // 3) PI to speed
        we_hat = (PLL_L1 * e) + (PLL_L2 * integ_e);

        // 4) speed clamp (electrical)
        //we_hat = clampf(we_hat, -PLL_WE_LIMIT, +PLL_WE_LIMIT);

        // 5) integrate to angle
        th_e_hat = wrap_pm_pi(th_e_hat + we_hat * PLL_TS);
    }

    // =======================
    // (B) build continuous electrical angle from filtered estimate
    // =======================
    float dth_hat = wrap_pm_pi(th_e_hat - th_e_hat_prev);
    th_e_hat_prev = th_e_hat;

    float th_e_cont = theta_e_cont_rad + dth_hat;
    theta_e_cont_rad = th_e_cont;

    // =======================
    // (C) mechanical position & speed from PLL states
    // =======================
    float th_m = th_e_cont / polepair;
    pos_mech_rad = th_m;

    // mech speed: from we_hat (no differentiation noise)
    float w_m_est = (we_hat / polepair); // mech rad/s

    // optional LPF on speed (keep your setting)
    w_m_hold = w_m_alpha * w_m_est + (1.0f - w_m_alpha) * w_m_hold_prev;
    w_m_hold_prev = w_m_hold;

    // =======================
    // (D) position error & PD
    // =======================
    float e_pos = pos_ref_mech_rad - th_m;

    P_part = pos_P * e_pos;
    D_part = -(pos_D * w_m_hold);

    float iq_cmd = flip_sign * (P_part + D_part);

    // saturate
    iq_cmd = (iq_cmd > IQ_REF_LIMIT) ? IQ_REF_LIMIT : (iq_cmd < -IQ_REF_LIMIT) ? -IQ_REF_LIMIT : iq_cmd;

    iq_ref = iq_cmd;
}*/

/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){  //ADC conversion complete callback, spend 5.2us to conversion
	if      (hadc->Instance == ADC1) done |= (1<<0);
	else if (hadc->Instance == ADC2) done |= (1<<1);
	else if (hadc->Instance == ADC3) done |= (1<<2);

	if(done == 7){
		done = 0;

		// alpha beta current calculation
		ic = current_A[0] - current_A_calibrated;  // A<->C mapping hardware issue
		ib = current_B[0] - current_B_calibrated;
		//ia = current_C[0] - current_C_calibrated;  // A<->C mapping hardware issue
		ia = -(ib+ic); // ia is very unstable. currnet_C has big jitter (+-10 LSB) (maybe hardware PCB problem. maybe charge pump?)

		// inv clarke
		i_alpha = ia * current_const;                   // i_alpha = 2/3 * (ia - (ib + ic)/2) = ia
		i_beta  = (ib - ic) * invsqrt3 * current_const; // i_beta  = 2/3 * (ib - ic) * sqrt(3)/2 = (ib - ic) * 1/sqrt(3)

		// digital LPF(mean sum)
		i_alpha_LPF = (i_alpha + i_alpha_prev) * 0.5;
		i_beta_LPF  = (i_beta  + i_beta_prev)  * 0.5;
		i_alpha_prev = i_alpha;
		i_beta_prev  = i_beta;

		// HFI start
		if(HFI_test){
			if(flag){

				// wait
				if(count++ == 9999){
					flag = 0;
					count = 0;
				}
			}
			else{
				// rotor angle estimation with HFI
				if(i == 1){  // one tick after V_injected flip
					if(have_prev_HFI){
						delta_i_alpha = i_alpha - i_alpha_prev_HFI;
						delta_i_beta  = i_beta  - i_beta_prev_HFI;

						if(V_injected > 0){
							delta_i_alpha = - delta_i_alpha;
							delta_i_beta  = - delta_i_beta;
						}

						//theta_e_est_rad = atan2f(delta_i_beta, delta_i_alpha);
						theta_e_est_rad = cordic_atan2_rad(delta_i_beta, delta_i_alpha);

						if(have_prev_estimation){
							float case1 = theta_e_est_rad;
							float case2 = theta_e_est_rad + PI;

							if(fabsf(wrap_pm_pi(case1 - theta_e_est_rad_prev)) > fabsf(wrap_pm_pi(case2 - theta_e_est_rad_prev))){
								theta_e_est_rad = case2;
								angle_flip = 1;
							}
							else{
								theta_e_est_rad = case1;
								angle_flip = 0;
							}
						}

						theta_e_est_rad = PosEst_PLL_Update_FromHFI(theta_e_est_rad);

						theta_e_est_rad_prev = theta_e_est_rad;

						have_prev_estimation = 1;

						estimated_electrical_angle = theta_e_est_rad * (180.0f / PI);
						theta_e_wrapped_rad = theta_e_est_rad;
					}

					i_alpha_prev_HFI = i_alpha;
					i_beta_prev_HFI  = i_beta;
					have_prev_HFI    = 1;
				}


				if(have_prev_HFI){
					delta_i_alpha = i_alpha - i_alpha_prev_HFI;
					delta_i_beta  = i_beta  - i_beta_prev_HFI;

					if(V_injected > 0){
						delta_i_alpha = - delta_i_alpha;
						delta_i_beta  = - delta_i_beta;
					}

					//theta_e_est_rad = atan2f(delta_i_beta, delta_i_alpha);
					theta_e_est_rad = cordic_atan2_rad(delta_i_beta, delta_i_alpha);

					if(have_prev_estimation){
						float case1 = theta_e_est_rad;
						float case2 = theta_e_est_rad + PI;

						if(fabsf(wrap_pm_pi(case1 - theta_e_est_rad_prev)) > fabsf(wrap_pm_pi(case2 - theta_e_est_rad_prev))){
							theta_e_est_rad = case2;
							angle_flip = 1;
						}
						else{
							theta_e_est_rad = case1;
							angle_flip = 0;
						}
					}

					theta_e_est_rad_prev = theta_e_est_rad;

					have_prev_estimation = 1;

					estimated_electrical_angle = theta_e_est_rad * (180.0f / PI);
					theta_e_wrapped_rad = theta_e_est_rad;
				}

				i_alpha_prev_HFI = i_alpha;
				i_beta_prev_HFI  = i_beta;
				have_prev_HFI    = 1;
				//here

				arm_sin_cos_f32(estimated_electrical_angle,&_sin,&_cos);

				if(count < 9999){ //estimate start, wait estimation converge
					V_alpha = V_injected * _cos;
					V_beta =  V_injected * _sin;
					count++;
				}
				else if(count < 79999){ // for torque control, get axis flip or not
					if(count < 40000){
						get_flip_start = theta_e_est_rad * flip_alpha + (1 - flip_alpha) * get_flip_start_prev;
						get_flip_start_prev = get_flip_start;

						V_alpha = 1.0f + V_injected * _cos;
						V_beta =  1.0f + V_injected * _sin;
					}
					else if(count < 70000){
						get_flip_end = theta_e_est_rad * flip_alpha + (1 - flip_alpha) * get_flip_end_prev;
						get_flip_end_prev = get_flip_end;

						V_alpha = -1.0f + V_injected * _cos;
						V_beta =  1.0f + V_injected * _sin;
					}
					else{
						if((get_flip_end - get_flip_start) > 0)
							flip_sign = -1;
						else
							flip_sign = 1;

						V_alpha = V_injected * _cos;
						V_beta  = V_injected * _sin;
					}
					count++;
				}
				else{
					// inv park, using high frequency rejected
					id =  i_alpha_LPF * _cos + i_beta_LPF * _sin;
					iq = -i_alpha_LPF * _sin + i_beta_LPF * _cos;

					//PI
					iq_err = iq_ref - iq;
					id_err = id_ref - id;
					iq_err_integral = iq_err_integral + iq_err;
					id_err_integral = id_err_integral + id_err;

					// Vq, Vd calculation
					Vq_unsat = (P * iq_err) + (I * iq_err_integral);
					Vd_unsat = (P * id_err) + (I * id_err_integral) + V_injected; // d axis injection

					Vd = (Vd_unsat > Vd_limit) ? Vd_limit : (Vd_unsat < -Vd_limit) ? -Vd_limit : Vd_unsat;

					arm_sqrt_f32((V_LIMIT*V_LIMIT) - (Vd*Vd), &Vq_limit);
					Vq_limit = Vq_limit * 0.9; // 0.9 for leave injection voltage
					Vq = (Vq_unsat > Vq_limit) ? Vq_limit : (Vq_unsat < -Vq_limit) ? -Vq_limit : Vq_unsat;

					iq_err_integral = iq_err_integral - Ka*(Vq_unsat - Vq);
					id_err_integral = id_err_integral - Ka*(Vd_unsat - Vd);

					// park
					V_alpha = Vd * _cos - Vq * _sin;
					V_beta  = Vd * _sin + Vq * _cos;

				}

				//uint32_t debug_time_3 = dwt_now();

				SVPWM(V_alpha, V_beta);

				if(i++ == 1){  // injection flip with 5 tick -> 20 kHz / (2 * 5tick) = 2kHz injection square wave voltage (i = 4) //now 10k (i = 1)
					i = 0;
					V_injected = -V_injected;
				}

				//V_injected = -V_injected;

				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
				//uint32_t debug_time_end = dwt_now();
				//real_us = cycles_to_us(debug_time_end - debug_time_start);

			}
		}
		else{
			SVPWM(0.0,0.0);
		}
	}
	return;
}
*/

/*
void SVPWM(float Valpha, float Vbeta){
	// MOSFET deadtime compensation (하는중)
    float Vmag; arm_sqrt_f32(Valpha*Valpha + Vbeta*Vbeta, &Vmag);

    if (Vmag < 1e-6f) { set_ccr(ARR_TICKS,ARR_TICKS,ARR_TICKS); return;}

    // M = Vmag / (Vdc/√3) = √3 * Vmag / Vdc
    float M = 1.73205080 * (Vmag / V_BUS);
    if (M > 0.99f) M = 0.99f;

    float ang = atan2f(Vbeta, Valpha); if (ang < 0) ang += 6.2831853f;
    uint8_t k = (uint8_t)(ang / SECTOR_RAD);
    if (k >= 6) k = 0;                // 랩 보정
    float phi = ang - k*SECTOR_RAD;

    float T1 = M * arm_sin_f32(SECTOR_RAD - phi);  // T1 = Vmag / (2/3*Vdc) * sin(60deg-phi) / sin(60deg) =  √3 * Vmag / Vdc * sin(60deg-phi) = M * sin(60deg-phi)
    float T2 = M * arm_sin_f32(phi);               // T2 = Vmag / (2/3*Vdc) * sin(phi) / sin(60deg)       =  √3 * Vmag / Vdc * sin(phi)       = M * sin(phi)

    int32_t t1 = (int32_t)(ARR_TICKS * T1);
    int32_t t2 = (int32_t)(ARR_TICKS * T2);

    int32_t t0 = (ARR_TICKS - t1 - t2) >> 1;
    if (t0 < 0) t0 = 0;

    uint16_t Ua, Ub, Uc;
    switch (k>5?5:k){
        case 0: Ua=t1+t2+t0; Ub=t2+t0;     Uc=t0;            break;
        case 1: Ua=t1+t0;    Ub=t1+t2+t0;  Uc=t0;            break;
        case 2: Ua=t0;       Ub=t1+t2+t0;  Uc=t2+t0;         break;
        case 3: Ua=t0;       Ub=t1+t0;     Uc=t1+t2+t0;      break;
        case 4: Ua=t2+t0;    Ub=t0;        Uc=t1+t2+t0;      break;
        default:Ua=t1+t2+t0; Ub=t0;        Uc=t1+t0;         break;
    }
    set_ccr(ARR_TICKS - Ua, ARR_TICKS - Ub, ARR_TICKS - Uc);
}*/

/*printf("%.3f, %.3f, %.3f, %.3f\n", cordic_atan2_rad(1,0), cordic_atan2_rad(0,1), cordic_atan2_rad(-1,0), cordic_atan2_rad(0,-1));
	 printf("45deg: %.3f\n", cordic_atan2_rad(1,1));     // ~0.785
	 printf("135:   %.3f\n", cordic_atan2_rad(1,-1));    // ~2.356
	 printf("-45:   %.3f\n", cordic_atan2_rad(-1,1));    // ~-0.785
	 printf("-135:  %.3f\n", cordic_atan2_rad(-1,-1));   // ~-2.356 (또는 +? 랩)
	 HAL_Delay(1000);*/
