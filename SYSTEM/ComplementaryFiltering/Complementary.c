#include "Complementary.h"
#include "math.h"
#include "usart.h"

#define PI 3.14159

extern char ret_words[200];
//float magx_max = 0.0f;
//float magy_max = 0.0f;
//float magz_max = 0.0f;
//float magx_min = 0.0f;
//float magy_min = 0.0f;
//float magz_min = 0.0f;
//float magx_offset = 0.0f;
//float magy_offset = 0.0f;
//float magz_offset = 0.0f;

float invSqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

void complementary_data_init(Fliter_Data* fliter_data){
	fliter_data->ready = false;
	fliter_data->gyroOffset[0] = 0;
	fliter_data->gyroOffset[1] = 0;
	fliter_data->gyroOffset[2] = 0;
}

void complementary_task(Fliter_Data* fliter_data,Fliter_Result_Data*out_data,MPU9250_DATD* in_data,float dt){
	
	static u16 offset_count = 0;
	static	float gyro_offsets_sum[3] = {0,0,0};
	float Rot_matrix[9] = {1.f,  0.0f,  0.0f, 0.0f,  1.f,  0.0f, 0.0f,  0.0f,  1.f };
	float acc[3] = {0.0f, 0.0f, 0.0f};
	float gyro[3] = {0.0f, 0.0f, 0.0f};
	float mag[3] = {0.0f, 0.0f, 0.0f};
	if(!fliter_data->ready){
		if(OFFSET_COUNT >= offset_count){
			gyro_offsets_sum[0] += in_data->gx;
            gyro_offsets_sum[1] += in_data->gy;
            gyro_offsets_sum[2] += in_data->gz;
			offset_count++;
					
			return;
		}
		
		fliter_data->gyroOffset[0] = gyro_offsets_sum[0]/offset_count;
        fliter_data->gyroOffset[1] = gyro_offsets_sum[1]/offset_count;
        fliter_data->gyroOffset[2] = gyro_offsets_sum[2]/offset_count;
		
		#ifdef TEST_COMPLEMENTARY		
//		sprintf(ret_words,"gyroOffset[0]:%f,gyroOffset[1]:%f,gyroOffset[2]:%f\r\n",fliter_data->gyroOffset[0],fliter_data->gyroOffset[1],fliter_data->gyroOffset[2]);
//		Uart1_SendString((u8*)ret_words);
//		sprintf(ret_words,"magx_offset:%f,magy_offset:%f,magz_offset:%f\r\n",magx_offset,magy_offset,magz_offset);
//		Uart1_SendString((u8*)ret_words);
		#endif
		offset_count=0;
        gyro_offsets_sum[0]=0;gyro_offsets_sum[1]=0;gyro_offsets_sum[2]=0;
        fliter_data->ready = 1;
	}
	
	gyro[0] = in_data->gx - fliter_data->gyroOffset[0];
	gyro[1] = in_data->gy - fliter_data->gyroOffset[1];
	gyro[2] = in_data->gz - fliter_data->gyroOffset[2];
	
	
	#ifdef TEST_COMPLEMENTARY		
	sprintf(ret_words,"gyro[0]:%f,gyro[1]:%f,gyro[2]:%f\r\n",gyro[0],gyro[1],gyro[2]);
	Uart1_SendString((u8*)ret_words);
	#endif
	
	acc[0] = in_data->ax;
	acc[1] = in_data->ay;
	acc[2] = in_data->az;

	mag[0] = in_data->mx;
	mag[1] = in_data->my;
	mag[2] = in_data->mz;
		
	complementary_fliter(gyro[0],gyro[1],gyro[2],acc[0],acc[1],acc[2],mag[1],mag[0],-mag[2],dt);
	
	// Convert q->R, This R converts inertial frame to body frame.
	Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
	Rot_matrix[1] = 2.f * (q1*q2 + q0*q3);	// 12
	Rot_matrix[2] = 2.f * (q1*q3 - q0*q2);	// 13
	Rot_matrix[3] = 2.f * (q1*q2 - q0*q3);	// 21
	Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
	Rot_matrix[5] = 2.f * (q2*q3 + q0*q1);	// 23
	Rot_matrix[6] = 2.f * (q1*q3 + q0*q2);	// 31
	Rot_matrix[7] = 2.f * (q2*q3 - q0*q1);	// 32
	Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33
	
	#ifdef TEST_COMPLEMENTARY 
	sprintf(ret_words,"Rot_matrix[0]:%f,Rot_matrix[1]:%f\r\n",Rot_matrix[0],Rot_matrix[1]);
	Uart1_SendString((u8*)ret_words);
	#endif
	
	out_data->euler[0] = (atan2f(Rot_matrix[5], Rot_matrix[8]) * 180) / PI;        //! Roll
    out_data->euler[1] = (-asinf(Rot_matrix[2]) * 180) / PI;	//! Pitch
    out_data->euler[2] = (atan2f(Rot_matrix[1], Rot_matrix[0]) * 180) /PI;		//! Yaw

//	out_data->euler[0] =  atan2f(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)*57.3;     
//    out_data->euler[1] =  asinf(2*q1*q3 - 2*q0*q2)*57.3;                                                          
//    out_data->euler[2] =  -atan2f(2*q1*q2 + 2*q0*q3, -2*q2*q2 -2*q3*q3 + 1)*57.3;
	out_data->q[0] = q0;
	out_data->q[1] = q1;
	out_data->q[2] = q2;
	out_data->q[3] = q3;

	return;
}


void complementary_fliter(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float dt){
		float recipNorm;
        float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

        // Make filter converge to initial solution faster
        // This function assumes you are in static position.
        // WARNING : in case air reboot, this can cause problem. But this is very unlikely happen.
        if(bFilterInit == 0) {
                complementary_init(ax,ay,az,mx,my,mz);
                bFilterInit = 1;
        }

        //! If magnetometer measurement is available, use it.
        if(!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
            float hx, hy, hz, bx, bz;
            float halfwx, halfwy, halfwz;

            // Normalise magnetometer measurement
            // Will sqrt work better? PX4 system is powerful enough
			recipNorm = invSqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;

            // Reference direction of Earth's magnetic field
			hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
			hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
			hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
			bx = sqrt(hx * hx + hy * hy);
			bz = hz;

            // Estimated direction of magnetic field
			halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
			halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
			halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

            // Error is sum of cross product between estimated direction and measured direction of field vectors
			halfex += (my * halfwz - mz * halfwy);
			halfey += (mz * halfwx - mx * halfwz);
			halfez += (mx * halfwy - my * halfwx);
        }

        //增加一个条件：  加速度的模量与G相差不远时。 0.75*G < normAcc < 1.25*G
        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
        {
                float halfvx, halfvy, halfvz;

                // Normalise accelerometer measurement
                //归一化，得到单位加速度
				recipNorm = invSqrt(ax * ax + ay * ay + az * az);
				ax *= recipNorm;
				ay *= recipNorm;
				az *= recipNorm;

                // Estimated direction of gravity and magnetic field
				halfvx = q1q3 - q0q2;
				halfvy = q0q1 + q2q3;
				halfvz = q0q0 - 0.5f + q3q3;

                // Error is sum of cross product between estimated direction and measured direction of field vectors
				halfex += ay * halfvz - az * halfvy;
				halfey += az * halfvx - ax * halfvz;
				halfez += ax * halfvy - ay * halfvx;
        }

        // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
        if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
                // Compute and apply integral feedback if enabled
                if(Ki > 0.0f) {
					gyro_bias[0] += Ki * halfex * dt;	// integral error scaled by Ki
					gyro_bias[1] += Ki * halfey * dt;
					gyro_bias[2] += Ki * halfez * dt;

                        // apply integral feedback
                        gx += gyro_bias[0];
                        gy += gyro_bias[1];
                        gz += gyro_bias[2];
                }
                else {
                        gyro_bias[0] = 0.0f;        // prevent integral windup
                        gyro_bias[1] = 0.0f;
                        gyro_bias[2] = 0.0f;
                }

                // Apply proportional feedback
                gx += Kp * halfex;
                gy += Kp * halfey;
                gz += Kp * halfez;
        }

        // Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
        //! q_k = q_{k-1} + dt*\dot{q}
        //! \dot{q} = 0.5*q \otimes P(\omega)
		dq0 = 0.5f*(-q1 * gx - q2 * gy - q3 * gz);
		dq1 = 0.5f*(q0 * gx + q2 * gz - q3 * gy);
		dq2 = 0.5f*(q0 * gy - q1 * gz + q3 * gx);
		dq3 = 0.5f*(q0 * gz + q1 * gy - q2 * gx); 

        q0 += dt*dq0;
        q1 += dt*dq1;
        q2 += dt*dq2;
        q3 += dt*dq3;

        // Normalise quaternion
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3; 
}

void complementary_init(float ax, float ay, float az, float mx, float my, float mz){

	float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll = atan2(-ay, -az);
    initialPitch = atan2(ax, -az);

    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);

    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}
