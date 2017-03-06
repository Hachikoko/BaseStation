#include "kalman.h"
#include "math.h"
#include "stdbool.h"

float roll,pitch,yaw;
struct Kalman kalmanX, kalmanY, kalmanZ; // Create the Kalman instances
bool kalman_init_flag = false;
int magX, magY, magZ;  //磁力计原始数据

double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only 只用陀螺仪计算角度
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter  用电磁计计算角度
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter    用kalman计算角度

void updata_roll(MPU9250_RAW_DATD*mpu9250_data){
	roll = atan2(mpu9250_data->ay,mpu9250_data->az) * RAD_TO_DEG;
    pitch = atan(-mpu9250_data->ax / sqrt(mpu9250_data->ay * mpu9250_data->ay + mpu9250_data->az * mpu9250_data->az)) * RAD_TO_DEG;
}

void updata_yaw(MPU9250_RAW_DATD*mpu9250_data){
	double rollAngle,pitchAngle,Bfy,Bfx;
	
	magX = mpu9250_data->my;
	magY = mpu9250_data->mx;
	magZ = -mpu9250_data->mz;
	
	rollAngle  = kalAngleX * DEG_TO_RAD;
    pitchAngle = kalAngleY * DEG_TO_RAD;

    Bfy = magZ * sin(rollAngle) - magY * cos(rollAngle);
    Bfx = magX * cos(pitchAngle) + magY * sin(pitchAngle) * sin(rollAngle) + magZ * sin(pitchAngle) * cos(rollAngle);
    yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;
    yaw *= -1;
}

void kalman_init(MPU9250_RAW_DATD*mpu9250_data){
	
	Init(&kalmanX);
	Init(&kalmanY);
	Init(&kalmanZ);
	
	updata_roll(mpu9250_data);
	updata_yaw(mpu9250_data);
	
	setAngle(&kalmanX,roll); // First set roll starting angle
    gyroXangle = roll;
    compAngleX = roll;
	
	setAngle(&kalmanY,pitch); // Then pitch
    gyroYangle = pitch;
    compAngleY = pitch;

    setAngle(&kalmanZ,yaw); // And finally yaw
    gyroZangle = yaw;
    compAngleZ = yaw;
}

void kalman_filter_main(float dt,MPU9250_RAW_DATD* mpu9250_data,Fliter_Result_Data* fliter_result_data){

	double gyroXrate,gyroYrate,gyroZrate;
	
	if(false == kalman_init_flag){
		kalman_init(mpu9250_data);
		kalman_init_flag = true;
	}
	
	updata_roll(mpu9250_data);
	gyroXrate = ((float)mpu9250_data->gx / 32760.0f)*8.7266465;     // Convert to deg/s    把陀螺仪的角加速度按照当初设定的量程转换为°/s
    gyroYrate = ((float)mpu9250_data->gy / 32760.0f)*8.7266465;     // Convert to deg/s
	if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        setAngle(&kalmanX,roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
    } else
    kalAngleX = getAngle(&kalmanX, roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
	
	if (fabs(kalAngleX) > 90)
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
    kalAngleY = getAngle(&kalmanY,pitch, gyroYrate, dt);
	
	
	
    updata_yaw(mpu9250_data);
    gyroZrate = ((float)mpu9250_data->gz / 32760.0f)*8.7266465;  // Convert to deg/s
    // This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
    if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
        setAngle(&kalmanZ,yaw);
        compAngleZ = yaw;
        kalAngleZ = yaw;
        gyroZangle = yaw;
    } else
    kalAngleZ = getAngle(&kalmanZ, yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter

	gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    gyroZangle += gyroZrate * dt;
	
	
	/* Estimate angles using complimentary filter */
    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
    compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;
	
	// Reset the gyro angles when they has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;
    if (gyroZangle < -180 || gyroZangle > 180)
        gyroZangle = kalAngleZ;
	
	fliter_result_data->euler[0] = kalAngleX;
	fliter_result_data->euler[1] = kalAngleY;
	fliter_result_data->euler[2] = kalAngleZ;
	
	
}
