/*
    センサ使用情報：https://qiita.com/Ninagawa_Izumi/items/8a4ed03fe8bb3cdcb8c4
*/
#include <Wire.h>
#include <SparkFunLSM9DS1.h>

#define LSM9DS1_M 0x1c  // 地磁気センサのアドレス
#define LSM9DS1_AG 0x6A // 加速度・ジャイロセンサのアドレス

#define EQ_POINT 0.1    // 平衡点（角度[rad]）
//#define EQ_POINT 0.3    // 平衡点（角度[rad]）
//#define EQ_POINT 0.45    // 平衡点（角度[rad]）
//#define EQ_POINT 0.39    // 平衡点（角度[rad]）
//#define EQ_POINT 0.35    // 平衡点（角度[rad]）
//#define POWER 200       // モータのパワー（0-255）

#define TS 20 // 遅延時間

/* センサ設定 */
LSM9DS1 imu;    // センサの定義
float imu_6dof[6]; //ax,ay,az,gx,gy,gz,mx,my,mz,temperature,roll,pitch,yaw
//float imu_9dof[13]; //ax,ay,az,gx,gy,gz,mx,my,mz,temperature,roll,pitch,yaw
float theta = 0;    // 角度計測値の初期値
float theta_p = 0;

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

    // 角度を計算
    theta = compleFilter( theta_p );    // 角度を計算（相補フィルタで補正）

    // 入力を計算
    int u = controller_PD( theta, theta_p );
    
    // 角度の更新
    theta_p = theta ;

    // シリアル送信
    Serial.print(slavePresent(LSM9DS1_AG)); // 存在判定結果を返す（デバッグ用）
    Serial.print(", "); // 
    Serial.print(theta);// 相補フィルタの計測角度を出力
    Serial.print(", "); // 
    Serial.print(calcThetaAcc());// 加速度センサからの計測角度を出力
    Serial.print(", "); // 
    Serial.print(saturatePow(u));// 制御入力を表示
    //Serial.print(u);// 制御入力を表示
    Serial.println();   // 改行

    // プログラムの遅延
    delay(TS);  // 10msにするとセンサ値が読めなかった
}

// センサの存在判定を行う関数
bool slavePresent(byte adr){
    Wire.beginTransmission(adr);    // 通信開始
    return( Wire.endTransmission() == 0);   // 通信終了してその結果を真偽値で返す
}
// 制御器関数
int controller_PD( float th, float th_p ){
    float e  = th + EQ_POINT ;   // 平衡点からの誤差の計算
    float ed = th - th_p ;  // 誤差の微分値を計算

    // PDにより入力生成
    float u = 150.0 * e + 300.0 * ed ;
    //float u = 150.0 * e + 400.0 * ed ; // 
    //float u = 250.0 * e + 250.0 * ed ; // 悪い
    //float u = 150.0 * e + 250.0 * ed ; // 相補フィルタゲイン変更
    //float u = 100.0 * e + 300.0 * ed ; // P低い
    //float u = 150.0 * e + 250.0 * ed ;
    // float u = 250.0 * e + 100.0 * ed ;
    //float u = 250.0 * e + 80.0 * ed ;
    //float u = 200.0 * e + 80.0 * ed ; // 少し有効
    //float u = 150.0 * e + 80.0 * ed ;   // ゲイン変更
    //float u = 100.0 * e + 60.0 * ed ;
    //float u = 100.0 * e + 40.0 * ed ;
    //float u = 100.0 * e + 5.0 * ed ;
    //float u = 50.0 * e + 1.0 * ed ;
    int up = int(u);    // 入力を整数型に変換
    
    // モータ駆動
    if( 0 < up ){   // upが負なら前進方向に駆動
        motorGo( up );
    }else if ( 0 > up ){ // upが正なら後進方向に駆動
        motorBack( -up );
    }else{  // upが0なら停止
        motorStop();
    }
    return up ;
}
// モータを止める関数
void motorStop(){
    analogWrite( AB_IN1, 0 );
    analogWrite( AB_IN2, 0 );
}
// モータ後進関数
void motorBack( int pow ){
    int u = saturatePow( pow ); // 入力飽和関数を通す
    analogWrite( AB_IN2, 0 );
    analogWrite( AB_IN1, u );
}
// モータ前進関数
void motorGo( int pow ){
    int u = saturatePow( pow ); // 入力飽和関数を通す
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
    return u ;
}
// 相補フィルタ
//  ジャイロセンサの積分値（角度）と，加速度センサから求めた角度の内分点を計算している
float compleFilter( float th_p ){
    //float th = 0.8 * ( th_p + imu_6dof[3] * 0.001 * (TS + 3) ) + 0.2 * calcThetaAcc() ;
    //float th = 0.85 * ( th_p + imu_6dof[3] * 0.001 * (TS + 3) ) + 0.15 * calcThetaAcc() ;
    //float th = 0.9 * ( th_p + imu_6dof[3] * 0.001 * (TS + 3) ) + 0.1 * calcThetaAcc() ;
    float th = 0.95 * ( th_p + imu_6dof[3] * 0.001 * (TS + 3) ) + 0.05 * calcThetaAcc() ;
    return th ;
}
// 加速度センサ値から角度を計算する関数
float calcThetaAcc(){
    // y軸とz軸の値からatan2で角度を計算
    float th = atan2( imu_6dof[2], imu_6dof[1] );
    return th ;
}
//IMUの値を取得する関数
void getIMU() {
    // 加速度センサが通信可能か判定する
    if(slavePresent(LSM9DS1_AG)){
        imu.readAccel();
        imu.readGyro();
    };
    //取得したデータを配列に格納
    imu_6dof[0] = imu.calcAccel(imu.ax);
    imu_6dof[1] = imu.calcAccel(imu.ay);
    imu_6dof[2] = imu.calcAccel(imu.az);
    imu_6dof[3] = imu.calcGyro(imu.gx);
    imu_6dof[4] = imu.calcGyro(imu.gy);
    imu_6dof[5] = imu.calcGyro(imu.gz);
}