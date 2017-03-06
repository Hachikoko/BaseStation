#include "kalman_filter.h"
#include "main.h"
#include "attitudeKalmanfilter.h"
#include "stdio.h"
#include "usart.h"

extern char ret_words[200];

static struct attitude_estimator_ekf_params ekf_params;




void kalman_filter_main(float dt,MPU9250_DATD data_9,float * euler){

	if(gyro_init_flag == false){
		if(gyro_init_count <= OFFSET_COUNT){
			gyro_offsets[0] += data_9.gx;
			gyro_offsets[1] += data_9.gy;
			gyro_offsets[2] += data_9.gz;	
			gyro_init_count++;
			return;
		}else{
			gyro_init_flag = true;
			gyro_offsets[0] /= OFFSET_COUNT;
			gyro_offsets[1] /= OFFSET_COUNT;
			gyro_offsets[2] /= OFFSET_COUNT;
			
			#ifdef  TEST_KALMAN
			sprintf(ret_words,"gyroOffset[0]:%f,gyroOffset[1]:%f,gyroOffset[2]:%f\r\n",gyro_offsets[0],gyro_offsets[1],gyro_offsets[2]);
			Uart1_SendString((u8*)ret_words);
			#endif
			
			
		}
		
		z_k[0] =  data_9.gx - gyro_offsets[0];
		z_k[1] =  data_9.gy - gyro_offsets[1];
		z_k[2] =  data_9.gz - gyro_offsets[2];

		z_k[3] =  data_9.ax;
		z_k[4] =  data_9.ay;
		z_k[5] =  data_9.az;
		
		z_k[6] = data_9.mx;
		z_k[7] = data_9.my;
		z_k[8] = data_9.mz;
		
		
		
		if(x_aposteriori_k_init_flag == false){
			x_aposteriori_k[0] = z_k[0];
			x_aposteriori_k[1] = z_k[1];
			x_aposteriori_k[2] = z_k[2];
			x_aposteriori_k[3] = 0.0f;
			x_aposteriori_k[4] = 0.0f;
			x_aposteriori_k[5] = 0.0f;
			x_aposteriori_k[6] = z_k[3];
			x_aposteriori_k[7] = z_k[4];
			x_aposteriori_k[8] = z_k[5];
			x_aposteriori_k[9] = z_k[6];
			x_aposteriori_k[10] = z_k[7];
			x_aposteriori_k[11] = z_k[8];
			
			x_aposteriori_k_init_flag = true;
			
			ekf_params.r[0] = 0.0008f;
			ekf_params.r[1] = 10000.0f;
			ekf_params.r[2] = 100.0f;
			ekf_params.r[3] = 0.0f;
			
			ekf_params.q[0] = 1e-4f;
			ekf_params.q[1] = 0.08f;
			ekf_params.q[2] = 0.009f;
			ekf_params.q[3] = 0.005f;
			ekf_params.q[4] = 0.0f;
		}
		#ifdef  TEST_KALMAN
		sprintf(ret_words,"before attitudeKalmanfilter\r\n");
		Uart1_SendString((u8*)ret_words);
		#endif
		attitudeKalmanfilter(updata_vector, dt, z_k, x_aposteriori_k, P_aposteriori_k, ekf_params.q, ekf_params.r,
							 euler, Rot_matrix, x_aposteriori, P_aposteriori);
		
		#ifdef  TEST_KALMAN
		sprintf(ret_words,"after attitudeKalmanfilter\r\n");
		Uart1_SendString((u8*)ret_words);
		#endif

		memcpy(P_aposteriori_k, P_aposteriori, sizeof(P_aposteriori_k));
		memcpy(x_aposteriori_k, x_aposteriori, sizeof(x_aposteriori_k));
		
		
		return;
		
		
	}
}
