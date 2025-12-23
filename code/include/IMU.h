/* ライブラリ導入 */
#include <Adafruit_BNO055.h>            // 9軸センサBNO055用のライブラリ
#include <Wire.h>

struct IMU_Data_Vector3{
    double x;
    double y;
    double z;
    //オイラー角の場合はx=Yaw,y=Pitch,z=Rollに対応
};

struct IMU_Data_All{
    // 角速度データ、加速度データ、磁気データの全てをまとめた構造体
    IMU_Data_Vector3 gyro;
    IMU_Data_Vector3 accel;
    IMU_Data_Vector3 mag;
    IMU_Data_Vector3 euler;
};

class IMU_Manager {
    private:
        Adafruit_BNO055 bno;
        float gyro_x_offset = 0.0, gyro_y_offset = 0.0, gyro_z_offset = 0.0;
        float accel_x_offset = 0.0, accel_y_offset = 0.0, accel_z_offset = 0.0;
        float mag_x_offset = 0.0, mag_y_offset = 0.0, mag_z_offset = 0.0;
        float imu_euler_offset_yaw = 0.0, imu_euler_offset_pitch = 0.0, imu_euler_offset_roll = 0.0;
    public:
        /* BNO055用変数 */
        IMU_Manager() : bno(55, 0x29, &Wire1) {}
        //初期化
        bool begin(){
            Wire1.begin();
            if(!bno.begin()){
                return false;
            }
            delay(500);
            IMU_Reset();
            return true;
        }
    void IMU_Reset(){
        gyro_x_offset = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).x();
        gyro_y_offset = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).y();
        gyro_z_offset = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).z();

        accel_x_offset = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER).x();
        accel_y_offset = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER).y();
        accel_z_offset = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER).z();

        mag_x_offset = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER).x();
        mag_y_offset = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER).y();
        mag_z_offset = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER).z();

        imu_euler_offset_yaw = bno.getVector(Adafruit_BNO055::VECTOR_EULER).x();
        imu_euler_offset_pitch = bno.getVector(Adafruit_BNO055::VECTOR_EULER).y();
        imu_euler_offset_roll = bno.getVector(Adafruit_BNO055::VECTOR_EULER).z();
    }
    //各種データ取得関数
    // 角速度データ取得 単位は[rad/s]
    IMU_Data_Vector3 Get_IMU_GYRO(){
        imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        return IMU_Data_Vector3{gyroscope.x() - gyro_x_offset, gyroscope.y() - gyro_y_offset, gyroscope.z() - gyro_z_offset};
    }

    // 加速度データ取得 単位は[m/s^2]
    IMU_Data_Vector3 Get_IMU_ACCEL(){
        imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        return IMU_Data_Vector3{accelermetor.x() - accel_x_offset, accelermetor.y() - accel_y_offset, accelermetor.z() - accel_z_offset};
    }

    // 磁気データ取得 単位は[μT]
    IMU_Data_Vector3 Get_IMU_MAG(){
        imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        return IMU_Data_Vector3{magnetometer.x() - mag_x_offset, magnetometer.y() - mag_y_offset, magnetometer.z() - mag_z_offset};
    }

    // オイラー角データ取得 単位は[degree]
    //xはYaw,yはPitch,zはRollに対応
    IMU_Data_Vector3 Get_IMU_EULER(){
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

        //0°～360°の範囲に補正
        float yaw = euler.x() - imu_euler_offset_yaw;
        float pitch = euler.y() - imu_euler_offset_pitch;
        float roll = euler.z() - imu_euler_offset_roll;
        if(yaw < 0) yaw += 360.0;
        else if(yaw >= 360.0) yaw -= 360.0;
        if(pitch < 0) pitch += 360.0;
        else if(pitch >= 360.0) pitch -= 360.0;
        if(roll < 0) roll += 360.0;
        else if(roll >= 360.0) roll -= 360.0;
        
        //-180°～180°の範囲に変換
        if(yaw > 180.0) yaw -= 360.0;
        if(pitch > 180.0) pitch -= 360.0;
        if(roll > 180.0) roll -= 360.0;
        return IMU_Data_Vector3{yaw, pitch, roll};
    }

    IMU_Data_All Get_IMU_All(){
        return {
            Get_IMU_GYRO(),
            Get_IMU_ACCEL(),
            Get_IMU_MAG(),
            Get_IMU_EULER()
        };
    }
};
