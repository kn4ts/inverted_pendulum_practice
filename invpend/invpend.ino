/*
    センサ使用情報：https://qiita.com/Ninagawa_Izumi/items/8a4ed03fe8bb3cdcb8c4
*/
#include <Wire.h>
#include <SparkFunLSM9DS1.h>

#define LSM9DS1_M 0x1c  // 地磁気センサのアドレス
#define LSM9DS1_AG 0x6A // 加速度・ジャイロセンサのアドレス

#define EQ_POINT 0.3    // 平衡点（角度[rad]）
#define POWER 200       // モータのパワー（0-255）

/* センサ設定 */
LSM9DS1 imu;    // センサの定義
float imu_9dof[13]; //ax,ay,az,gx,gy,gz,mx,my,mz,temperature,roll,pitch,yaw
float theta = 0;    // 角度計測値の初期値

/* アクチュエータ設定 */
int AB_IN1 = 6; // AI1, BI1
int AB_IN2 = 5; // AI2, BI2

void setup(){
    // シリアル通信を開始
    Serial.begin(115200);
    // I2C通信を開始
    Wire.begin();
    //IMUの起動を判定
    if (imu.begin(LSM9DS1_AG, LSM9DS1_M, Wire) == false){
        Serial.println("Failed to communicate with LSM9DS1.");
        while (1);
    }
    // アクチュエータを初期値に（モータ停止指令）
    motorStop();
}

void loop(){
    // 計測
    getIMU();   // センサ値更新

    // シリアル送信
    Serial.print(millis()); // 内部クロック時間を表示（デバッグ用）
    Serial.print(", ");
    Serial.print("here!, ");// デバッグ用表示

    theta = calcTheta() + EQ_POINT ;    // 角度を計算（平衡点で補正）

    // 入力生成（仮）とモータ駆動
    if( 0 < theta ){
        motorGo( POWER );   // POWER値で前進
        //motorGo( Pgain * theta );
    }else if ( 0 > theta )
    {
        motorBack( POWER ); // POWER値で後進
        //motorBack( Pgain * theta );
    }else{
        motorStop();    // 平衡点ではストップ
    }

    // シリアル送信
    Serial.print(slavePresent(LSM9DS1_AG)); // 存在判定結果を返す（デバッグ用）
    Serial.print(", "); // 
    Serial.print(theta);// 計測角度を出力
    Serial.println();   // 改行

    // プログラムの遅延
    delay(20);  // 10msにするとセンサ値が読めなかった
}

// センサの存在判定を行う関数
bool slavePresent(byte adr){
    Wire.beginTransmission(adr);    // 通信開始
    return( Wire.endTransmission() == 0);   // 通信終了してその結果を真偽値で返す
}

// モータを止める関数
void motorStop(){
    analogWrite( AB_IN1, 0 );
    analogWrite( AB_IN2, 0 );
}
// モータ後進関数
void motorBack( int pow ){
    int u = saturatePow( pow );
    analogWrite( AB_IN1, u );
    analogWrite( AB_IN2, 0 );
}
// モータ前進関数
void motorGo( int pow ){
    int u = saturatePow( pow );
    analogWrite( AB_IN1, 0 );
    analogWrite( AB_IN2, u );
}

// モータドライバへの入力を飽和させる関数
int saturatePow( int pow ){
    int u = pow;
    if( 255 < pow ){
        u = 255;
    }else if ( 0 > pow ){
        u = 0;
    }
    return u;
}

// 加速度センサ値から角度を計算する関数
float calcTheta(){
    // y軸とz軸の値からatan2で角度を計算
    float th = atan2( imu_9dof[2], imu_9dof[1] );
    return th;
}
void getIMU() {//IMUの値を取得する関数
    // 加速度センサが通信可能か判定する
    if(slavePresent(LSM9DS1_AG)){
        //if ( imu.gyroAvailable() )  imu.readGyro();
        if ( imu.accelAvailable() ) imu.readAccel();
    };
    //if ( imu.gyroAvailable() )  imu.readGyro();
    //if ( imu.magAvailable() )   imu.readMag();
    //if ( imu.tempAvailable() )  imu.readTemp();
    //取得したデータを配列に格納する。roll, pitch, yawの計算は適当
    imu_9dof[0] = imu.calcAccel(imu.ax);
    imu_9dof[1] = imu.calcAccel(imu.ay);
    imu_9dof[2] = imu.calcAccel(imu.az);
    //imu_9dof[3] = imu.calcGyro(imu.gx);
    //imu_9dof[4] = imu.calcGyro(imu.gy);
    //imu_9dof[5] = imu.calcGyro(imu.gz);
    //imu_9dof[6] = imu.calcMag(imu.mx);
    //imu_9dof[7] = imu.calcMag(imu.my);
    //imu_9dof[8] = imu.calcMag(imu.mz);
    //imu_9dof[9] = imu.temperature;
    //※以下の算出方法はすごく適当な仮の値なので全く参考にしないでください。
    //imu_9dof[10] = atan2(imu.ay, imu.az) * 180.0 / PI; //roll
    //imu_9dof[11] = atan2(-imu.ax, sqrt(imu.ay * imu.ay + imu.az * imu.az)) * 180.0 / PI; //pitch
    //imu_9dof[12] = atan2(imu.mx, imu.my) * 180.0 / PI; //yaw
}