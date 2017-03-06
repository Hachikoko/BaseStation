#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

#include "stdbool.h"
#include "attitudeKalmanfilter.h"
#include "Complementary.h"



struct attitude_estimator_ekf_params {
	float r[9];
	float q[12];
	float roll_off;
	float pitch_off;
	float yaw_off;
	float mag_decl;
	int acc_comp;
};



//准备输入参数
uint8_T updata_vector[3] = {1,1,1};
float z_k[9] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 9.81f, 0.2f, -0.2f, 0.2f};					/**< Measurement vector */
float x_aposteriori_k[12];		/**< states */
float P_aposteriori_k[144] = {100.f, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
				     0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
				     0,   0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,   0,
				     0,   0,   0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,
				     0,   0,   0,   0,  100.f,  0,   0,   0,   0,   0,   0,   0,
				     0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,   0,   0,
				     0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,   0,
				     0,   0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,
				     0,   0,   0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,
				     0,   0,   0,   0,   0,   0,   0,   0,  0.0f, 100.0f,   0,   0,
				     0,   0,   0,   0,   0,   0,   0,   0,  0.0f,   0,   100.0f,   0,
				     0,   0,   0,   0,   0,   0,   0,   0,  0.0f,   0,   0,   100.0f,
				    }; /**< init: diagonal matrix with big values */

float x_aposteriori[12];
float P_aposteriori[144];

					
float Rot_matrix[9] = {1.f,  0,  0,
			             0,  1.f,  0,
			             0,  0,  1.f
			          };		/**< init: identity matrix */
//配置参数
float gyro_offsets[3] = {0.0f,0.0f,0.0f};
bool gyro_init_flag = false;
int gyro_init_count = 0;
bool x_aposteriori_k_init_flag = false;

void kalman_filter_main(float dt,MPU9250_DATD raw_data,float * euler);



#endif //kalman_filter.h
