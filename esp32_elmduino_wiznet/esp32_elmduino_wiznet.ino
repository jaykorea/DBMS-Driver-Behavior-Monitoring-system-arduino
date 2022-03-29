//------------------------------------------------------------cat.M1----------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------------------------//
#include "Arduino.h"
#include "at_cmd_parser.h"
#include "aws_iot_config.h"
#include "awscerts.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "Ticker.h"

#define RESP_OK                            "OK\r\n"
#define RET_OK                             1
#define RET_NOK                            -1
#define DEBUG_ENABLE                       1
#define DEBUG_DISABLE                      0
#define ON                                 1
#define OFF                                0

#define PWR_PIN 32
#define RST_PIN 33

#define MAX_BUF_SIZE                1024

#define BG96_APN_PROTOCOL_IPv4             1
#define BG96_APN_PROTOCOL_IPv6             2
#define BG96_DEFAULT_TIMEOUT               1000
#define BG96_WAIT_TIMEOUT                  3000
#define BG96_CONNECT_TIMEOUT               15000
#define BG96_SEND_TIMEOUT                  500
#define BG96_RECV_TIMEOUT                  500

#define BG96_APN_PROTOCOL                  BG96_APN_PROTOCOL_IPv6
#define WM_N400MSE_DEFAULT_BAUD_RATE       115200
#define BG96_PARSER_DELIMITER              "\r\n"

#define CATM1_APN_SKT                      "lte-internet.sktelecom.com"

#define CATM1_DEVICE_NAME_BG96             "BG96"
#define DEVNAME                             CATM1_DEVICE_NAME_BG96

#define LOGDEBUG(x, ...)             if(CATM1_DEVICE_DEBUG == DEBUG_ENABLE) { Serial.printf("\r\n[%s] ", DEVNAME);  Serial.printf((x), ##__VA_ARGS__); }
#define MYPRINTF(x, ...)             {Serial.printf("\r\n[MAIN] ");  Serial.printf((x), ##__VA_ARGS__);}

// Sensors
#define MBED_CONF_IOTSHIELD_SENSOR_CDS     A0
#define MBED_CONF_IOTSHIELD_SENSOR_TEMP    A1

// Debug message settings
#define BG96_PARSER_DEBUG                  DEBUG_DISABLE
#define CATM1_DEVICE_DEBUG                 DEBUG_ENABLE

#define REQUESTED_PERIODIC_TAU            "10100101"
#define REQUESTED_ACTIVE_TIME             "00100100"

/* MQTT */
#define MQTT_EOF                                    0x1A
#define MQTT_QOS0                                   0
#define MQTT_QOS1                                   1
#define MQTT_QOS2                                   2
#define MQTT_RETAIN                                 0

/* SSL/TLS */
// Ciphersuites
#define BG96_TLS_RSA_WITH_AES_256_CBC_SHA           "0x0035"
#define BG96_TLS_RSA_WITH_AES_128_CBC_SHA           "0x002F"
#define BG96_TLS_RSA_WITH_RC4_128_SHA               "0x0005"
#define BG96_TLS_RSA_WITH_RC4_128_MD5               "0x0004"
#define BG96_TLS_RSA_WITH_3DES_EDE_CBC_SHA          "0x000A"
#define BG96_TLS_RSA_WITH_AES_256_CBC_SHA256        "0x003D"
#define BG96_TLS_ECDHE_RSA_WITH_RC4_128_SHA         "0xC011"
#define BG96_TLS_ECDHE_RSA_WITH_3DES_EDE_CBC_SHA    "0xC012"
#define BG96_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA     "0xC013"
#define BG96_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA     "0xC014"
#define BG96_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256  "0xC027"
#define BG96_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA384  "0xC028"
#define BG96_TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256  "0xC02F"
#define BG96_TLS_SUPPORT_ALL                        "0xFFFF"

// SSL/TLS version
#define BG96_TLS_VERSION_SSL30                      0   // SSL3.0
#define BG96_TLS_VERSION_TLS10                      1   // TLS1.0
#define BG96_TLS_VERSION_TLS11                      2   // TLS1.1
#define BG96_TLS_VERSION_TLS12                      3   // TLS1.2
#define BG96_TLS_VERSION_ALL                        4

/* Debug message settings */
#define BG96_PARSER_DEBUG           DEBUG_DISABLE
#define CATM1_DEVICE_DEBUG          DEBUG_ENABLE 


/* MQTT Sample */
// MQTT connection state
enum {
  MQTT_STATE_OPEN = 0,
  MQTT_STATE_CONNECT,
  MQTT_STATE_CONNECTED,
  MQTT_STATE_DISCON
};

ATCmdParser m_parser = ATCmdParser(&Serial2); //wiznet shield

/* BG96 Config for connect to AWS IoT */
#define AWS_IOT_BG96_SSLTLS_VERSION              4           // 4: All
#define AWS_IOT_BG96_SSLTLS_SECLEVEL             2           // 2: Manage server and client authentication if requested by the remote server
#define AWS_IOT_BG96_SSLTLS_IGNORELOCALTIME      1           // 1: Ignore validity check for certification
#define AWS_IOT_BG96_SSLTLS_CIPHERSUITE          BG96_TLS_RSA_WITH_AES_256_CBC_SHA

        /* AWS IoT MQTT Client */    
    char aws_iot_sub_topic[128] = {0, };
    char aws_iot_pub_topic[128] = {0, };
    char aws_iot_sub_topic1[128] = {0, };
    char aws_iot_pub_topic1[128] = {0, };
    char aws_iot_sub_topic2[128] = {0, };
    char aws_iot_pub_topic2[128] = {0, };    
    char buf_mqtt_topic[128] = {0, };
    char buf_mqtt_topic1[128] = {0, };
    char buf_mqtt_topic2[128] = {0, };
    char buf_mqtt_recv[AWS_IOT_MQTT_RX_BUF_LEN] = {0, };
    char buf_mqtt_send[AWS_IOT_MQTT_TX_BUF_LEN] = {0, };
    char buf_mqtt_recv1[AWS_IOT_MQTT_RX_BUF_LEN] = {0, };
    char buf_mqtt_send1[AWS_IOT_MQTT_TX_BUF_LEN] = {0, };
    char buf_mqtt_recv2[AWS_IOT_MQTT_RX_BUF_LEN] = {0, };
    char buf_mqtt_send2[AWS_IOT_MQTT_TX_BUF_LEN] = {0, };
    int mqtt_len = 0;
    int mqtt_len1 = 0;
    int mqtt_len2 = 0;
    int mqtt_msgid = 0;
    int mqtt_msgid1 = 0;
    int mqtt_masgid2 = 0; 

unsigned long getLocationTime = 0;

char dateBuf[30];
static char last_dateBuf[30];
char utcBuf[30];
static char last_utcBuf[30];
char latBuf[50];
static char last_latBuf[30] = "37.40002";
char lonBuf[50];
static char last_lonBuf[30] = "126.94254";
char timestampBuf[20] = "12345678";


typedef struct gps_data_t {
  float utc;      // hhmmss.sss
  float lat;      // latitude. (-)dd.ddddd
  float lon;      // longitude. (-)dd.ddddd
  float hdop;     // Horizontal precision: 0.5-99.9
  float altitude; // altitude of antenna from sea level (meters)
  int fix;        // GNSS position mode 2=2D, 3=3D
  float cog;      // Course Over Ground ddd.mm
  float spkm;     // Speed over ground (Km/h) xxxx.x
  float spkn;     // Speed over ground (knots) xxxx.x
  char date[7];   // data: ddmmyy
  int nsat;       // number of satellites 0-12
} gps_data;

gps_data gps_info;


static int8_t counter_AccelYp;
static int8_t counter_Fuel;
static int8_t counter_Rpm;
static float efficiencyMark;
static int8_t counter_AccelX;
static int8_t counter_AccelYm;
static int8_t counter_AccelYm2;
static int8_t counter_Time;
static int8_t counter_Kph;
static float safetyMark;
static float drivingMark = 10.0;
static float averagedrivingMark = 10.0;

static float AFR = 14.70;

float pre_f_accelX;
float pre_f_accelY;
float pre_instantFuelConsume;
uint32_t pre_rpm;
uint32_t pre_runTime;
int32_t pre_kph;
float averageFuelConsume = 12.0;
float instantFuelConsume = 12.0;


bool flag_aws_publish = true; //flag
Ticker flipper; //timer

char car_state[30] = "";
int pwrCheck;
int rstCheck;
bool errorvaluecheck = true;

//--------------------------------------------------ELMDUINO_SD_TFT-------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------------------------//

#include <NewToneLib.h>
MyTone t(false);
                             
#include <SPI.h>
#include <SdFat.h>
SdFat sd; //sd카드
#define SPI_SPEED SD_SCK_MHZ(16)
const uint8_t chipSelect = 5; //due - 52 , mega - 53
//const int chipSelect = 4;   //cs pin of SD card shield, MKR NB1500,

File dataFile;   // the logging file
char filename[] = "LOG00000.CSV";

unsigned long c_time = 0;
unsigned long e_time = 0;

int HOUR = 0;
int MIN = 0;
int SEC = 0; // 자체 시간 시,분,초

int E_HOUR = 0;
int E_MIN = 0;
int E_SEC = 0; // 자체 시간 시,분,초

//int samplingTime = 1;  //this variable is interval(in Seconds) at which you want to log the data to SD card.
//int duration = 300;     //this variable is duration(in Minutes) which is the total time for which you want to log data.


#include "ELMduino.h"
#include <SPI.h>
#include <stdio.h>


//#define ELM_PORT Serial1
HardwareSerial ELM_PORT(1);
ELM327 myELM327;

#define BUZZER 12
int red = 25;
int blue = 26;
int green = 27;

uint32_t rpm = 0;
uint32_t kph = 0;
uint32_t engineLoad = 0;
uint16_t runTime = 0;
uint8_t fuelType = 0;
int32_t oilTemp = 0;
uint32_t relativePedalPos = 0;
uint32_t throttle = 0;
uint32_t relativeThrottle = 0;
int32_t intakeAirTemp = 0;
uint32_t fuelLevel = 0;
float mafRate = 0; //uint32_T
uint8_t obdStandards = 0;
uint16_t distTravelWithMIL = 0;
uint16_t distSinceCodesCleared = 0;
uint32_t ctrlModVoltage = 0;
int16_t ambientAirTemp = 0;
uint32_t manifoldPressure = 0;
int32_t engineCoolantTemp = 0;
uint32_t commandedThrottleActuator = 0; 
float commandedAirFuelRatio = 0.0; //elm327 데이터 변수

const int beepFrequencyS1 = 1500; //1000Hz, 연결성공시 부저 주파수1
const int beepFrequencyS2 = 2500; //2500Hz, 연결성공시 부저 주파수2
const int beepFrequencyF = 750; //750Hz, 연결실패시 부저 주파수
const int beepDurationS = 250; //0.25sec ,성공시 지속시간, (31~65535 Hz)
const int beepDurationF = 250; //0.25sec ,실패시 지속시간, 

#define cur_myELM327_STATUS curS;
#define pre_myELM327_STATUS preS;

//-------------------------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------ELMDUINO_SD_TFT-------------------------------------------------------//



//------------------------------------------------------------GYRO-----------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------------------------//
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
MPU6050 accelgyro(0x68); // <--use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

 /* =========================================================================
    NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
    when using Serial.write(buf, len). The Teapot output uses this method.
    The solution requires a modification to the Arduino USBAPI.h file, which
    is fortunately simple, but annoying. This will be fixed in the next IDE
    release. For more info, see these links:

    http://arduino.cc/forum/index.php/topic,109987.0.html
    http://code.google.com/p/arduino/issues/detail?id=958
  * ========================================================================= */



  // uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
  // quaternion components in a [w, x, y, z] format (not best for parsing
  // on a remote host such as Processing or something though)
  //#define OUTPUT_READABLE_QUATERNION

  // uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
  // (in degrees) calculated from the quaternions coming from the FIFO.
  // Note that Euler angles suffer from gimbal lock (for more info, see
  // http://en.wikipedia.org/wiki/Gimbal_lock)
  //#define OUTPUT_READABLE_EULER

  // uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
  // pitch/roll angles (in degrees) calculated from the quaternions coming
  // from the FIFO. Note this also requires gravity vector calculations.
  // Also note that yaw/pitch/roll angles suffer from gimbal lock (for
  // more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
  #define OUTPUT_READABLE_YAWPITCHROLL

  // uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
  // components with gravity removed. This acceleration reference frame is
  // not compensated for orientation, so +X is always +X according to the
  // sensor, just without the effects of gravity. If you want acceleration
  // compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
  //#define OUTPUT_READABLE_REALACCEL

  // uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
  // components with gravity removed and adjusted for the world frame of
  // reference (yaw is relative to initial orientation, since no magnetometer
  // is present in this case). Could be quite handy in some cases.
    #define OUTPUT_READABLE_WORLDACCEL

  // uncomment "OUTPUT_TEAPOT" if you want output that matches the
  // format used for the InvenSense teapot demo
  //#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 13  // use pin 2 on Arduino Uno & most boards
//#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
//bool blinkState = false;
#define CALIBRATION_TIMEOUT 20000

// MPU control/status vars
bool dmpReady = true;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// calibration int
int buffersize = 1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)


int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;

float ACCEL_X;
float ACCEL_Y;
float ACCEL_Z;
float GYRO_X;
float GYRO_Y;
float GYRO_Z;


int mean_accX, mean_accY, mean_accZ, mean_gyroX, mean_gyroY, mean_gyroZ, state = 0;
int accX_offset, accY_offset, accZ_offset, gyroX_offset, gyroY_offset, gyroZ_offset;

float f_pitch;
float f_roll;
float f_accelX;
float f_accelY;

unsigned long pre_TimeoutCheck = 0;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//-------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------GYRO-----------------------------------------------------------------//


void setup()
  //------------------------------------------------------------GYRO-----------------------------------------------------------------//
  //-------------------------------------------------------------------------------------------------------------------------------------//
{
    {
        // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
#endif

        // COMMENT NEXT LINE IF YOU ARE USING ARDUINO DUE
        //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Leonardo measured 250kHz.

        // initialize serial communication
        // (115200 chosen because it is required for Teapot Demo output, but it's
        // really up to you depending on your project)
        Serial.begin(115200);
        //while (!Serial); // wait for Leonardo enumeration, others continue immediately

        // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
        // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
        // the baud timing being too misaligned with processor ticks. You must use
        // 38400 or slower in these cases, or use some kind of external separate
        // crystal solution for the UART timer.

        // initialize device
        Serial.println(F("Initializing I2C devices..."));

        mpu.initialize();
        pinMode(INTERRUPT_PIN, OUTPUT);

        delay(100);

        // verify connection
        Serial.println(F("Testing device connections..."));
        Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

        // wait for ready
        /*Serial.println(F("\nSend any character to begin DMP programming and demo: "));
        while (Serial.available() && Serial.read()); // empty buffer
        while (!Serial.available());                 // wait for data
        while (Serial.available() && Serial.read()); // empty buffer again*/
     if (mpu.testConnection() == true ) {
        // load and configure the DMP
        Serial.println(F("Initializing DMP..."));
        delay(100);

        //reset offsets
        accelgyro.setXAccelOffset(0);
        accelgyro.setYAccelOffset(0);
        accelgyro.setZAccelOffset(0);
        accelgyro.setXGyroOffset(0);
        accelgyro.setYGyroOffset(0);
        accelgyro.setZGyroOffset(0);

        while (state < 3)
        {

            if (state == 0)
            {
                meansensors();
                state++;
                Serial.println("\nReading sensors for first time...");
                delay(1000);
            }
            if (state == 1)
            {
                calibration();
                Serial.println("\nCalculating offsets...");
                state++;
                delay(1000);
            }
            if (state == 2)
            {
                meansensors();
                Serial.println("\nFINISHED!");
                Serial.println("\nSensor readings with offsets:\t");
                state++;
            }
            
            delay(100);
        }

        float ACCEL_X = accX_offset;
        float ACCEL_Y = accY_offset;
        float ACCEL_Z = accZ_offset;
        float GYRO_X = gyroX_offset;
        float GYRO_Y = gyroY_offset;
        float GYRO_Z = gyroZ_offset;

        devStatus = mpu.dmpInitialize();

        // supply your own gyro offsets here, scaled for min sensitivity
        mpu.setXGyroOffset(GYRO_X);
        Serial.print("gyroX_offset = "); Serial.println(GYRO_X);
        mpu.setYGyroOffset(GYRO_Y);
        Serial.print("gyroY_offset = "); Serial.println(GYRO_Y);
        mpu.setZGyroOffset(GYRO_Z);
        Serial.print("gyroZ_offset = "); Serial.println(GYRO_Z);
        mpu.setXAccelOffset(ACCEL_X);
        Serial.print("accelX_offset = "); Serial.println(ACCEL_X);
        mpu.setYAccelOffset(ACCEL_Y);
        Serial.print("accelY_offset = "); Serial.println(ACCEL_Y);
        mpu.setZAccelOffset(ACCEL_Z);
        Serial.print("accelZ_offset = "); Serial.println(ACCEL_Z);

        delay(500);

        // make sure it worked (returns 0 if so)
        if (devStatus == 0) {
            // Calibration Time: generate offsets and calibrate our MPU6050
            mpu.CalibrateAccel(6);
            mpu.CalibrateGyro(6);
            Serial.println();
            mpu.PrintActiveOffsets();
            // turn on the DMP, now that it's ready
            Serial.println(F("Enabling DMP..."));
            mpu.setDMPEnabled(true);

            // enable Arduino interrupt detection
            Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
            Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
            Serial.println(F(")..."));
            attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
            mpuIntStatus = mpu.getIntStatus();

            // set our DMP Ready flag so the main loop() function knows it's okay to use it
            Serial.println(F("DMP ready! Waiting for first interrupt..."));
            dmpReady = true;

            // get expected DMP packet size for later comparison
            packetSize = mpu.dmpGetFIFOPacketSize();

            delay(100);
        }
     }
        else {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));

            delay(1500);
        }

    }

 //-------------------------------------------------------------------------------------------------------------------------------------//
 //------------------------------------------------------------GYRO-----------------------------------------------------------------//


 //--------------------------------------------------ELMDUINO_SD_TFT-------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------------------------//

    {
        pinMode(red, OUTPUT);
        pinMode(green, OUTPUT);
        pinMode(blue, OUTPUT);
        pinMode(BUZZER, OUTPUT);
        pinMode(PWR_PIN, OUTPUT);
        pinMode(RST_PIN, OUTPUT);

//#if LED_BUILTIN
//        pinMode(LED_BUILTIN, OUTPUT);
//        digitalWrite(LED_BUILTIN, LOW);
//#endif
        
        ELM_PORT.begin(38400, SERIAL_8N1, 4, 2);

        digitalWrite(red, HIGH);
        digitalWrite(green, HIGH);
        digitalWrite(blue, LOW);

        if (!myELM327.begin(ELM_PORT))
        {   
          unsigned long ELM327_TimeoutCheck = 0;  

          ELM327_TimeoutCheck = millis();
          
          Serial.println(F("ELM327 Connection Failed"));

            do
            {   

              Serial.print( (millis() - ELM327_TimeoutCheck) / 1000 );
              if ( millis() - ELM327_TimeoutCheck >= 6000 )
              { ESP.restart(); }
              
                digitalWrite(red, LOW);
                digitalWrite(green, HIGH);
                digitalWrite(blue, HIGH);
                t.tone(BUZZER, beepFrequencyF, beepDurationF);
                
                delay(650); //ELM327 연결 실패시 0.65초 간격으로 Fail 부저
            } while (1);
        }

        Serial.println(F("ELM327 Connected"));

        digitalWrite(green, LOW);
        digitalWrite(red, HIGH);
        digitalWrite(blue, HIGH);
        t.tone(BUZZER, beepFrequencyS1, beepDurationS);
        delay(500);
        t.tone(BUZZER, beepFrequencyS2, beepDurationS);
        delay(500);
        //ELM327 연결 성공시 Success 부저 울림, led green ON
    }

    Serial.print("Initializing SD card...");

    digitalWrite(green, HIGH);
    digitalWrite(red, HIGH);
    digitalWrite(blue, LOW);
    delay(500);
    // see if the card is present and can be initialized:
    if (!sd.begin(chipSelect, SPI_SPEED))
    {
        //sd.initErrorHalt();
        Serial.println("Card failed, or not present");

        digitalWrite(green, HIGH);
        digitalWrite(red, LOW);
        digitalWrite(blue, HIGH);

        t.tone(BUZZER, beepFrequencyF, beepDurationF);

        //return;
    }
    
    else if (sd.begin(chipSelect, SPI_SPEED))
    {
    digitalWrite(green, LOW);
    digitalWrite(red, HIGH);
    digitalWrite(blue, HIGH);

    t.tone(BUZZER, beepFrequencyS2, beepDurationS);
    delay(250);
    Serial.println("card initialized.");
    

    // create a new file
    for (unsigned int k = 0; k < 10000; k++)
    {
        
        filename[4] = ((k % 10000) / 1000) + '0';//천자리
        filename[5] = ((k % 1000) / 100) + '0';//백자리
        filename[6] = ((k % 100) / 10) + '0'; //십자리
        filename[7] = k % 10 + '0'; //일자리
        if (!sd.exists(filename))
        {
            // only open a new file if it doesn't exist
            dataFile = sd.open(filename, FILE_WRITE);
            break;  // leave the loop!
        }
    }
 }

    if (!dataFile)
    { //alert user
        Serial.println("couldnt create file");

        digitalWrite(green, HIGH);
        digitalWrite(red, LOW);
        digitalWrite(blue, HIGH);

        t.tone(BUZZER, beepFrequencyF, beepDurationF);
        delay(300);
        t.tone(BUZZER, beepFrequencyF, beepDurationF);
    }
  
    Serial.print("Logging to: ");
    Serial.println(filename);

    delay(300);

    dataFile = sd.open(filename, FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile)
    {
        dataFile.println("Time,ELM327_Status,RunTime,Rpm,Kph,EngineLoad,throttle,InstantFuelConsume,counter_AccelYp,counter_Fuel,counter_Rpm,efficiencyMark,counter_AccelX,counter_AccelYm,AccelYm2,counter_Time,counter_Kph,safetyMark,drivingMark,averagedrivingMark,f_pitch,f_roll,f_accelX,f_accelY");
        dataFile.print(HOUR);
        dataFile.print("h");
        dataFile.print("_");
        dataFile.print(MIN);
        dataFile.print("m");
        dataFile.print("_");
        dataFile.print(SEC);
        dataFile.print("s");
        dataFile.print(",");
        dataFile.println("FIRST BOOT!!!");
        dataFile.close();
    }
    // check availble space on SD Card
    uint32_t freeKB = sd.vol()->freeClusterCount();
    freeKB *= sd.vol()->blocksPerCluster() / 2;
    Serial.print("Free space KB: ");
    Serial.println(freeKB);
    uint32_t freeMB = freeKB / 1024;
    Serial.print("Free space in MB: ");
    Serial.println(freeMB);

    if (freeKB <= 500)
    {
        digitalWrite(green, HIGH);
        digitalWrite(red, LOW);
        digitalWrite(blue, HIGH);

        Serial.println("LOW SPACE!!!");

        t.tone(BUZZER, beepFrequencyF, beepDurationF);
        delay(300);
    }

    

//-------------------------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------ELMDUINO_SD_TFT-------------------------------------------------------------------//

//-----------------------------------------------BG_96_MODULE & AWS_IOT----------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------------------------------//
  {
    if (bg96_DeviceRST(rstCheck) == 1 ) {
    
  aws_flip();

  char buf[100];
  char buf1[40];

  // put your setup code here, to run once:
  //serialPcInit();
  catm1DeviceInit();

  MYPRINTF("Waiting for Cat.M1 Module Ready...\r\n");

  waitCatM1Ready();

  MYPRINTF("System Init Complete\r\n");

  MYPRINTF("WIZnet IoT Shield for Arduino");
  MYPRINTF("LTE Cat.M1 Version");
  MYPRINTF("=================================================");
  MYPRINTF(">> Target Board: WIoT-QC01 (Quectel BG96)");
  MYPRINTF(">> Sample Code:  AWS IoT Pub/Sub (Built-in TLS)");
  MYPRINTF("=================================================\r\n");

  for (int i = 0; i < 10; i++)
  {
    if (initStatus() == true) {
      delay(100);
      break;
    }
    else {
      LOGDEBUG("Please Check your H/W Status\r\n");
    }
    delay(1000);
  }

  setEchoStatus_BG96(false);
   
  getUsimStatus_BG96();
   
  getNetworkStatus_BG96();

  checknSetApn_BG96(CATM1_APN_SKT);
  
  MYPRINTF("[FILE] Save and check AWS certificates\r\n");    

  setContextActivate_BG96();
  
  int mqtt_state = MQTT_STATE_OPEN;
  
 /* Erase BG96 file storage */   
    if(eraseFileStorageAll_BG96() == RET_OK) {
        MYPRINTF("[FILE] Erase BG96 storage complete\r\n");
    };

    /* Store AWS IoT certificate files to BG96 storage */    
    saveFileToStorage_BG96(AWS_IOT_ROOT_CA_FILENAME, aws_iot_rootCA, strlen(aws_iot_rootCA));
    
    saveFileToStorage_BG96(AWS_IOT_CERTIFICATE_FILENAME, aws_iot_certificate, strlen(aws_iot_certificate));
    
    saveFileToStorage_BG96(AWS_IOT_PRIVATE_KEY_FILENAME, aws_iot_private_key, strlen(aws_iot_private_key));
        
#if 0    
    dumpFileList_BG96(); // file list dump
#endif
   
    MYPRINTF("[SSL/TLS] Set BG96 SSL/TLS configuration\r\n")

    /* BG96 SSL/TLS config */    
    // Set AWS IoT Certificate files
    setTlsCertificatePath_BG96("cacert", AWS_IOT_ROOT_CA_FILENAME);             // Root CA
    setTlsCertificatePath_BG96("clientcert", AWS_IOT_CERTIFICATE_FILENAME);     // Client certificate
    setTlsCertificatePath_BG96("clientkey", AWS_IOT_PRIVATE_KEY_FILENAME);      // Client privatekey

    // Set SSL/TLS config  
    setTlsConfig_sslversion_BG96(AWS_IOT_BG96_SSLTLS_VERSION);    
    setTlsConfig_ciphersuite_BG96(AWS_IOT_BG96_SSLTLS_CIPHERSUITE);    
    setTlsConfig_seclevel_BG96(AWS_IOT_BG96_SSLTLS_SECLEVEL);    
    setTlsConfig_ignoreltime_BG96(AWS_IOT_BG96_SSLTLS_IGNORELOCALTIME);


    /* BG96 MQTT config: SSL/TLS enable */
    setMqttTlsEnable_BG96(true);    
        
    MYPRINTF("[MQTT] Connect to AWS IoT \"%s:%d\"\r\n", AWS_IOT_MQTT_HOST, AWS_IOT_MQTT_PORT);
    
   flipper.attach(1.0,aws_flip); // Publish messages every 1 second

    if (setGpsOnOff_BG96(ON) == RET_OK) {
    MYPRINTF("GPS On\r\n")
#if 0
    if (setGpsOnOff_BG96(OFF) == RET_OK) {
      MYPRINTF("GPS Off\r\n")
    }
#endif
 }  else {
    MYPRINTF("GPS On failed\r\n")
  } 
  
  }
  else 
  Serial.println("bg96_ON Failed");
 }
 
    //bool subscribe_complete = false; 

    sprintf(aws_iot_sub_topic, "$aws/things/%s/shadow/update/accepted", AWS_IOT_MY_THING_NAME);
    sprintf(aws_iot_pub_topic, "$aws/things/%s/shadow/update", AWS_IOT_MY_THING_NAME);
    sprintf(aws_iot_pub_topic1, "$aws/things/%s/shadow/update", AWS_IOT_MY_THING_NAME);
    sprintf(aws_iot_pub_topic2, "dt/cardata/logger/%s/car-data", AWS_IOT_MY_THING_NAME);

}


void loop()
{

  //------------------------------------------------------------GYRO-----------------------------------------------------------------//
  //-------------------------------------------------------------------------------------------------------------------------------------//
    // if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) // Get the Latest packet 
    { 

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        Serial.print("quat\t");
        Serial.print(q.w);
        Serial.print("\t");
        Serial.print(q.x);
        Serial.print("\t");
        Serial.print(q.y);
        Serial.print("\t");
        Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        Serial.print("euler\t");
        Serial.print(euler[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(euler[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        /*Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / M_PI); //yaw
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI); //pitch
        Serial.print("\t");
        Serial.print(ypr[2] * 180 / M_PI); //roll*/
        /*
          mpu.dmpGetAccel(&aa, fifoBuffer);
          Serial.print("\tRaw Accl XYZ\t");
          Serial.print(aa.x);
          Serial.print("\t");
          Serial.print(aa.y);
          Serial.print("\t");
          Serial.print(aa.z);
          mpu.dmpGetGyro(&gy, fifoBuffer);
          Serial.print("\tRaw Gyro XYZ\t");
          Serial.print(gy.x);
          Serial.print("\t");
          Serial.print(gy.y);
          Serial.print("\t");
          Serial.print(gy.z);
         Serial.println();*/

#endif

#ifdef OUTPUT_READABLE_REALACCEL
        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        Serial.print("areal\t");
        Serial.print(aaReal.x);
        Serial.print("\t");
        Serial.print(aaReal.y);
        Serial.print("\t");
        Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
       /* Serial.print("aworld\t");
        Serial.print(aaWorld.x); // X accel
        Serial.print("\t");
        Serial.print(aaWorld.y); // Y accel
        Serial.print("\t");
        Serial.println(aaWorld.z); // Z accel*/
#endif

#ifdef OUTPUT_TEAPOT
        // display quaternion values in InvenSense Teapot demo format:
        teapotPacket[2] = fifoBuffer[0];
        teapotPacket[3] = fifoBuffer[1];
        teapotPacket[4] = fifoBuffer[4];
        teapotPacket[5] = fifoBuffer[5];
        teapotPacket[6] = fifoBuffer[8];
        teapotPacket[7] = fifoBuffer[9];
        teapotPacket[8] = fifoBuffer[12];
        teapotPacket[9] = fifoBuffer[13];
        Serial.write(teapotPacket, 14);
        teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif

        f_pitch = (float)(ypr[1] * 180 / M_PI);
        f_roll = (float)(ypr[2] * 180 / M_PI);
        f_accelX = ((float)aaWorld.x / 8192);
        f_accelY = ((float)aaWorld.y / 8192);

        
        Serial.println(f_pitch);
        Serial.println(f_roll);
        Serial.println(f_accelX);
        Serial.println(f_accelY);
        Serial.println("자이로상태 갱신");  

    }

//-------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------GYRO-----------------------------------------------------------------//
    

//----------------------------------------------------ELM DATA ON TFT-----------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------------------//
    static int8_t cur_myELM327_STATUS; // 현재상태 정적변수
    static int8_t pre_myELM327_STATUS; // 이전상태 정적변수

    float tempRPM = myELM327.rpm();
    uint32_t tempVEHICLE_SPEED = myELM327.kph();
    float tempENGINE_LOAD = myELM327.engineLoad();
    uint16_t tempRUN_TIME_SINCE_ENGINE_START = myELM327.runTime();
    uint8_t tempFUEL_TYPE = myELM327.fuelType();
    float tempENGINE_OIL_TEMP = myELM327.oilTemp();
    float tempENGINE_COOLANT_TEMP = myELM327.engineCoolantTemp();
    float tempRELATIVE_ACCELERATOR_PEDAL_POS = myELM327.relativePedalPos();
    float tempTHROTTLE_POSITION = myELM327.throttle();
    float tempCOMMANDED_THROTTLE_ACTUATOR = myELM327.commandedThrottleActuator();
    float tempRELATIVE_THROTTLE_POSITION = myELM327.relativeThrottle();
    float tempINTAKE_AIR_TEMP = myELM327.intakeAirTemp();
    uint8_t tempINTAKE_MANIFOLD_ABS_PRESSURE = myELM327.manifoldPressure();
    float tempFUEL_TANK_LEVEL_INPUT = myELM327.fuelLevel();
    uint8_t tempOBD_STANDARDS = myELM327.obdStandards();
    float tempCONTROL_MODULE_VOLTAGE = myELM327.ctrlModVoltage();
    float tempAMBIENT_AIR_TEMP = myELM327.ambientAirTemp();
    uint16_t tempDISTANCE_TRAVELED_WITH_MIL_ON = myELM327.distTravelWithMIL();
    uint16_t tempDIST_TRAV_SINCE_CODES_CLEARED = myELM327.distSinceCodesCleared();
    float tempMAF_FLOW_RATE = myELM327.mafRate();
    float tempFUEL_AIR_MANDED_EQUIV_RATIO = myELM327.commandedAirFuelRatio();



    c_time = millis() / 1000; //시간

    SEC = c_time % 60;
    MIN = (c_time / 60) % 60;
    HOUR = (c_time / (60 * 60)) % 24;


    for (byte i = 0; i < myELM327.recBytes; i++)
        Serial.write(myELM327.payload[i]);

    curS = myELM327.status; //현재상태 갱신

    switch (curS) {
      case ELM_SUCCESS:
      strcpy(car_state, "ELM_SUCCESS");
      break;
      
      case ELM_NO_RESPONSE:
      strcpy(car_state, "ELM_NO_RESPONSE");
      break; 

      case ELM_BUFFER_OVERFLOW:
      strcpy(car_state, "ELM_BUFFER_OVERFLOW");
      break;

      case ELM_UNABLE_TO_CONNECT:
      strcpy(car_state, "ELM_UNABLE_TO_CONNECT");
      break;

      case ELM_NO_DATA:
      strcpy(car_state, "ELM_NO_DATA");
      break;

      case ELM_STOPPED:
      strcpy(car_state, "ELM_STOPPED");
      break;

      case ELM_TIMEOUT:
      strcpy(car_state, "ELM_TIMEOUT");
      break;

      default:
      strcpy(car_state, "UNKNOWN_ERROR");
    }
/*
    Serial.print("ELM상태 : ");
    Serial.println(myELM327.status);
    Serial.print("이전상태 : ");
    Serial.println(preS);
    Serial.print("현재상태 : ");
    Serial.println(curS); // 상태체크 */

    if ( (curS != preS) && (sd.begin(chipSelect, SPI_SPEED)) ) // 이전상태와 현재상태가 다를시 새파일 생성
        createNewfile();

    preS = curS; // 이전상태 현재상태로 갱신

     /* byte B = myELM327.responseByte_0;
     byte A = myELM327.responseByte_1;
     elmduino.cpp 파일에서 사용하기
     단일 바이트 반환시 A = responseByte_0
     2개 바이트 반환시  A = responseByte_1, B = responseByte_0 으로 지정띠 */

     if (curS == ELM_SUCCESS) {
      
    Serial.println(F("ELM327 data representing....."));
    
    if (!sd.begin(chipSelect, SPI_SPEED))
    {

      digitalWrite(green, LOW);
      digitalWrite(red, HIGH);
      digitalWrite(blue, LOW);
    }

    else if (sd.begin(chipSelect, SPI_SPEED))
    {

      digitalWrite(green, LOW);
      digitalWrite(red, HIGH);
      digitalWrite(blue, HIGH);
    }

    
    runTime = (uint16_t)tempRUN_TIME_SINCE_ENGINE_START; //엔진켜진시점 이후 운행시간
    fuelType = (int32_t)tempFUEL_TYPE; //사용 연료정보
    rpm = (uint32_t)tempRPM; //차량 RPM
    kph = (int32_t)tempVEHICLE_SPEED; //차량속도
    engineLoad = (uint32_t)tempENGINE_LOAD; //엔진부하
    oilTemp = (int32_t)tempENGINE_OIL_TEMP; //오일온도
    engineCoolantTemp = (int32_t)tempENGINE_COOLANT_TEMP; //엔진 냉각수 온도
    throttle = (uint32_t)tempTHROTTLE_POSITION; //스로틀 포지션
    relativeThrottle = (uint32_t)tempRELATIVE_THROTTLE_POSITION; //상대 스로틀 포지션
    commandedThrottleActuator = (uint32_t)tempCOMMANDED_THROTTLE_ACTUATOR; //스로틀 엑츄에이터
    intakeAirTemp = (int32_t)tempINTAKE_AIR_TEMP; //흡입공기 온도
    mafRate = (float)tempMAF_FLOW_RATE; //공기유량
    manifoldPressure = (uint8_t)tempINTAKE_MANIFOLD_ABS_PRESSURE; //흡기매니폴드 절대압력
    ambientAirTemp = (int16_t)tempAMBIENT_AIR_TEMP; //외기온도
    distTravelWithMIL = (uint16_t)tempDISTANCE_TRAVELED_WITH_MIL_ON; //경고등 점등이후 주행거리
    distSinceCodesCleared = (uint16_t)tempDIST_TRAV_SINCE_CODES_CLEARED; //DTC 소거후 주행거리
    fuelLevel = (uint32_t)tempFUEL_TANK_LEVEL_INPUT; //연료레벨
    ctrlModVoltage = (uint32_t)tempCONTROL_MODULE_VOLTAGE; //컨트롤 모듈 전압
    obdStandards = (uint8_t)tempOBD_STANDARDS; //OBD 정보 - 수치값 wikipedia 검색
    commandedAirFuelRatio = (float)tempFUEL_AIR_MANDED_EQUIV_RATIO;

    Serial.print("commandedAirFuelRatio : ");
    Serial.println(commandedAirFuelRatio);

    Serial.print("AFR : ");
    Serial.println(AFR);

    //instantFuelConsume = ((mafRate*36000)/(kph*commandedAirFuelRatio*AFR*770)); //1L/100km 단위변환 
    instantFuelConsume = ((kph*AFR*commandedAirFuelRatio*770)/(mafRate*3600)); //km/l
    //instantFuelConsume = kph*1/3600*1/mafRate*AFR*770*commandedAirFuelRatio;
    Serial.print("instantFuelConsume : ");
    Serial.println(instantFuelConsume);

    if (errorValueReset() == true) {Serial.println("Error Value Reset!"); }
    
    if (fuelType == 1)
    static float AFR = 14.7;
    else if (fuelType == 2)
    static float AFR = 6.4;
    else if (fuelType == 3)
    static float AFR = 9.0;
    else if (fuelType == 4)
    static float AFR = 14.6;
    else if (fuelType == 5)
    static float AFR = 15.5;
    else if (fuelType == 6)
    static float AFR = 17.2;

    LogToSDcard(); // 시리얼모니터 확인 및 sd 카드 저장함수
    
     }

    else { 
      printError(curS);
      }

      if ( (rpm >100) && (pre_rpm > 100) && (curS == ELM_SUCCESS ) )//알고리즘 실행 조건 수정필요
      {
        
        fuelAlgorithm();        
        accelYpAlgorithm();
        rpmAlgorithm();
        efficiencyMarkAlgorithm();

        accelXAlgorithm();
        accelYmAlgorithm();
        accelYmAlgorithm2();
        timeAlgorithm();
        kphAlgorithm();
        safetyMarkAlgorithm();

        drivingMarkAlgorithm();
        averagedrivingMarkAlgorithm();
      }
      pre_DataUpdate();
      delay(10);
    
    {
      if ( (getGpsLocation_BG96(gps_info) == RET_OK) && (flag_aws_publish == true) ) {
        //MYPRINTF("Get GPS information >>>");
        //sprintf((char *)utcBuf, "gps_info - utc: %6.3f", gps_info.utc);
        //MYPRINTF(utcBuf)             // utc: hhmmss.sss
        //sprintf((char *)latBuf, "gps_info - lat: %2.5f", gps_info.lat);
        //MYPRINTF(latBuf)             // latitude: (-)dd.ddddd
        //sprintf((char *)lonBuf, "gps_info - lon: %2.5f", gps_info.lon);
        //MYPRINTF(lonBuf)             // longitude: (-)dd.ddddd
        //MYPRINTF("gps_info - hdop: %2.1f", gps_info.hdop)           // Horizontal precision: 0.5-99.9
        //MYPRINTF("gps_info - altitude: %2.1f", gps_info.altitude)   // altitude of antenna from sea level (meters)
        //MYPRINTF("gps_info - fix: %d", gps_info.fix)                // GNSS position mode: 2=2D, 3=3D
        //MYPRINTF("gps_info - cog: %3.2f", gps_info.cog)             // Course Over Ground: ddd.mm
        //MYPRINTF("gps_info - spkm: %4.1f", gps_info.spkm)           // Speed over ground (Km/h): xxxx.x
        //MYPRINTF("gps_info - spkn: %4.1f", gps_info.spkn)           // Speed over ground (knots): xxxx.x
        //MYPRINTF("gps_info - date: %s", gps_info.date)              // data: ddmmyy
        //MYPRINTF("gps_info - nsat: %d\r\n", gps_info.nsat)          // number of satellites: 0-12


       //char* last_utcBuf = utcBuf;
       //char* last_latBuf = latBuf;
       //char* last_lonBuf = lonBuf;
       //char* last_dateBuf = dateBuf;
       strcpy(last_utcBuf, utcBuf);
       strcpy(last_latBuf, latBuf);
       strcpy(last_lonBuf, lonBuf);
       strcpy(last_dateBuf, dateBuf);
       Serial.print("last_utcBuf 값 : ");
       Serial.println(last_utcBuf);
       Serial.print("last_latBuf 값 : ");
       Serial.println(last_latBuf);
       Serial.print("last_lonBuf 값 : ");
       Serial.println(last_lonBuf);
       Serial.print("last_dateBuf 값 : ");
       Serial.println(last_dateBuf);
       
      } else {
        //MYPRINTF("Failed to get GPS information\r\n");
        delay(10);
      }
      delay(100);
    }

       if(aws_iot_connection_process() == MQTT_STATE_CONNECTED) {                        
          
          // MQTT Subscribe
          if(flag_aws_publish == true) {                
              if(setMqttSubscribeTopic_BG96(aws_iot_sub_topic, 1, MQTT_QOS1) == RET_OK) {
                  MYPRINTF("[MQTT] Topic subscribed: \"%s\"\r\n", aws_iot_sub_topic);
              }
          }
           /*if (onetimecall == 0) {
            if(sendMqttPublishMessage_BG96(aws_iot_pub_topic1, MQTT_QOS1, MQTT_RETAIN, buf_mqtt_send1, mqtt_len1) == RET_OK) {
                  MYPRINTF("[MQTT] Message published: \"%s\", Message: %s\r\n", aws_iot_pub_topic1, buf_mqtt_send1);
              }
              onetimecall++;
          }*/
          
          // MQTT Publish
          if(flag_aws_publish == true) {                
              
              mqtt_len = sprintf(buf_mqtt_send, "{\"state\":{\"reported\":{\"name\":\"%s\",\"enabled\":\"%s\",\"geo\":{\"latitude\":\"%s\",\"longitude\":\"%s\"}}}}", AWS_IOT_MY_THING_NAME, car_state , last_latBuf, last_lonBuf);          
              if(sendMqttPublishMessage_BG96(aws_iot_pub_topic, MQTT_QOS1, MQTT_RETAIN, buf_mqtt_send, mqtt_len) == RET_OK) {
                  MYPRINTF("[MQTT] Message published: \"%s\", Message: %s\r\n", aws_iot_pub_topic, buf_mqtt_send);
              }

               delay(100);

              mqtt_len2 = sprintf(buf_mqtt_send2, "{\"pitch\": %f,\"roll\": %f,\"accelX\": %f,\"accelY\": %f,\"rpm\": %d,\"kph\": %d,\"engineLoad\": %d,\"runTime\": %d,\"fuelType\": %d,\"oilTemp\": %d,\"relativePedalPos\": %d,\"throttle\": %d,\"relativeThrottle\": %d,\"intakeAirTemp\": %d,\"fuelLevel\": %d,\"mafRate\": %f,\"obdStandards\": %d,\"distTravelWithMIL\": %d,\"distSinceCodesCleared\": %d,\"ctrlModVoltage\": %d,\"ambientAirTemp\": %d,\"manifoldPressure\": %d,\"engineCoolantTemp\": %d,\"commandedThrottleActuator\": %d,\"timestamp\": %s}", f_pitch, f_roll, f_accelX, f_accelY, rpm, kph, engineLoad, runTime, fuelType, oilTemp, relativePedalPos, throttle, relativeThrottle, intakeAirTemp, fuelLevel, mafRate, obdStandards, distTravelWithMIL, distSinceCodesCleared, ctrlModVoltage, ambientAirTemp, manifoldPressure, engineCoolantTemp, commandedThrottleActuator, timestampBuf);          
              if(sendMqttPublishMessage_BG96(aws_iot_pub_topic2, MQTT_QOS1, MQTT_RETAIN, buf_mqtt_send2, mqtt_len2) == RET_OK) {
                  MYPRINTF("[MQTT] Message published: \"%s\", Message: %s\r\n", aws_iot_pub_topic2, buf_mqtt_send2);
              }                

          }
            
          // MQTT message received
          if(checkRecvMqttMessage_BG96(buf_mqtt_topic, &mqtt_msgid, buf_mqtt_recv) == RET_OK) {
              MYPRINTF("[MQTT] Message arrived: Topic \"%s\" ID %d, Message %s\r\n", buf_mqtt_topic, mqtt_msgid, buf_mqtt_recv);                
          }
      }  

       flag_aws_publish = false; // flag clear

       delay(1);
  }

//----------------------------------------------------ELM DATA-----------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------------------//


void printError(int8_t curS)
{

    if (!sd.begin(chipSelect, SPI_SPEED))
    {
        digitalWrite(red, LOW);
        digitalWrite(green, HIGH);
        digitalWrite(blue, HIGH); //printERROR 시 red led ON

        t.tone(BUZZER, beepFrequencyF, beepDurationF);
    }
    else if (sd.begin(chipSelect, SPI_SPEED))
    {

        digitalWrite(red, LOW);
        digitalWrite(green, HIGH);
        digitalWrite(blue, LOW); 

         t.tone(BUZZER, beepFrequencyF, 0);
    }

    if (curS == ELM_NO_RESPONSE)
    {
        Serial.println(F("ERROR: ELM_NO_RESPONSE"));
    }
    else if (curS == ELM_BUFFER_OVERFLOW)
    {
        Serial.println(F("ERROR: ELM_BUFFER_OVERFLOW"));
    }
    else if (curS == ELM_UNABLE_TO_CONNECT)
    {
        Serial.println(F("ERROR: ELM_UNABLE_TO_CONNECT"));
    }
    else if (curS == ELM_NO_DATA)
    {
        Serial.println(F("ERROR: ELM_NO_DATA"));
    }
    else if (curS == ELM_STOPPED)
    {
        Serial.println(F("ERROR: ELM_STOPPED"));
    }
    else if (curS == ELM_TIMEOUT)
    {
        Serial.println(F("ERROR: ELM_TIMEOUT"));
    }
    else if (curS == ELM_TIMEOUT)
    {
        Serial.println(F("ERROR: ELM_GENERAL_ERROR"));
    }


    LogToSDcardError(curS);

   
    delay(100);
}


void LogToSDcard()
   {
    
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    dataFile = sd.open(filename, FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile)
    {
        Serial.println(F("ELM327 Data Logging..."));
        dataFile.print(HOUR);
        dataFile.print("h");
        dataFile.print("_");
        dataFile.print(MIN);
        dataFile.print("m");
        dataFile.print("_");
        dataFile.print(SEC);
        dataFile.print("s");
        dataFile.print(",");

        dataFile.print(F("ELM_SUCCES"));
        dataFile.print(",");

        dataFile.print(runTime / 60);
        dataFile.print(",");

        dataFile.print(rpm);
        dataFile.print(",");

        dataFile.print(kph);
        dataFile.print(",");

        dataFile.print(engineLoad);
        dataFile.print(",");

        dataFile.print(throttle);
        dataFile.print(",");

        dataFile.print(instantFuelConsume);
        dataFile.print(",");

        dataFile.print(counter_AccelYp);
        dataFile.print(",");

        dataFile.print(counter_Fuel);
        dataFile.print(",");

        dataFile.print(counter_Rpm);
        dataFile.print(",");

        dataFile.print(efficiencyMark);
        dataFile.print(",");

        dataFile.print(counter_AccelX);
        dataFile.print(",");

        dataFile.print(counter_AccelYm);
        dataFile.print(",");

        dataFile.print(counter_AccelYm2);
        dataFile.print(",");

        dataFile.print(counter_Time);
        dataFile.print(",");

        dataFile.print(counter_Kph);
        dataFile.print(",");

        dataFile.print(safetyMark);
        dataFile.print(",");

        dataFile.print(drivingMark);
        dataFile.print(",");

        dataFile.print(averagedrivingMark);
        dataFile.print(",");

        dataFile.print(f_pitch);
        dataFile.print(",");

        dataFile.print(f_roll);
        dataFile.print(",");

        dataFile.print(f_accelX);
        dataFile.print(",");

        dataFile.print(f_accelY);
        dataFile.println(",");




        dataFile.close();

        delay(25);

    }

    //if the file isn't open, pop up an error:
    /*else
    {
        Serial.println("error opening LOG_XXXX.txt");
        delay(100);
    }*/
}


void LogToSDcardError(int8_t curS)
{
    Serial.println(F("ELM_ERROR LOGGED"));

    e_time = millis() / 1000; //에러타임

    E_SEC = e_time % 60;
    E_MIN = (e_time / 60) % 60;
    E_HOUR = (e_time / (60 * 60)) % 24;

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.

         dataFile = sd.open(filename, FILE_WRITE);
         if (dataFile)
         {
             if (curS != ELM_SUCCESS)
             {

                 dataFile.print(E_HOUR);
                 dataFile.print("h");
                 dataFile.print("_");
                 dataFile.print(E_MIN);
                 dataFile.print("m");
                 dataFile.print("_");
                 dataFile.print(E_SEC);
                 dataFile.print("s");
                 dataFile.print(",");
             }


             if (curS == ELM_NO_RESPONSE)
             
                 dataFile.println(F("ERROR: ELM_NO_RESPONSE"));
             
             else if (curS == ELM_BUFFER_OVERFLOW)
             
                 dataFile.println(F("ERROR: ELM_BUFFER_OVERFLOW")); 
             
             else if (curS == ELM_UNABLE_TO_CONNECT)
             
                 dataFile.println(F("ERROR: ELM_UNABLE_TO_CONNECT"));
             
             else if (curS == ELM_NO_DATA)
             
                 dataFile.println(F("ERROR: ELM_NO_DATA"));
             
             else if (curS == ELM_STOPPED)
             
                 dataFile.println(F("ERROR: ELM_STOPPED")); 
             
             else if (curS == ELM_TIMEOUT)
             
                 dataFile.println(F("ERROR: ELM_TIMEOUT")); 
            
             else if (curS == ELM_TIMEOUT)
             
                 dataFile.println(F("ERROR: ELM_GENERAL_ERROR")); 
             


             dataFile.close();

             delay(5000); //1.5초 마다 tft 스크린 및 부저 표시.작동
         }

    
}


void createNewfile()
{
    dataFile.close();

    for (unsigned int k = 0; k < 100000; k++)
    {
        filename[3] = k / 10000 + '0';//만자리
        filename[4] = ((k % 10000) / 1000) + '0';//천자리
        filename[5] = ((k % 1000) / 100) + '0';//백자리
        filename[6] = ((k % 100) / 10) + '0'; //십자리
        filename[7] = k % 10 + '0'; //일자리
        if (!sd.exists(filename))
        {
            // only open a new file if it doesn't exist
            dataFile = sd.open(filename, FILE_WRITE);
            break;  // leave the loop!
        }
        delay(1);
    }

    dataFile = sd.open(filename, FILE_WRITE);
    if (dataFile)
    {
    Serial.print("Logging to: ");
    Serial.print(filename);
    Serial.print(F("<<---------------------------"));
    Serial.println(F("Created New File..."));
    
       dataFile.println("Time,ELM327_Status,RunTime,Rpm,Kph,EngineLoad,throttle,InstantFuelConsume,counter_AccelYp,counter_Fuel,counter_Rpm,efficiencyMark,counter_AccelX,counter_AccelYm,AccelYm2,counter_Time,counter_Kph,safetyMark,drivingMark,averagedrivingMark,f_pitch,f_roll,f_accelX,f_accelY");
        //csv 엑셀 윗라인 항목 표시띠

        dataFile.close();
    }
}



//-----------------------------------------------CALIBRATION ROUTINE-----------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------//

void meansensors() {
    long i = 0, buff_accX = 0, buff_accY = 0, buff_accZ = 0, buff_gyroX = 0, buff_gyroY = 0, buff_gyroZ = 0;

    while (i < (buffersize + 101)) {
        // read raw accel/gyro measurements from device
        accelgyro.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

        if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
            buff_accX = buff_accX + accX;
            buff_accY = buff_accY + accY;
            buff_accZ = buff_accZ + accZ;
            buff_gyroX = buff_gyroX + gyroX;
            buff_gyroY = buff_gyroY + gyroY;
            buff_gyroZ = buff_gyroZ + gyroZ;
        }
        if (i == (buffersize + 100)) {
            mean_accX = buff_accX / buffersize;
            mean_accY = buff_accY / buffersize;
            mean_accZ = buff_accZ / buffersize;
            mean_gyroX = buff_gyroX / buffersize;
            mean_gyroY = buff_gyroY / buffersize;
            mean_gyroZ = buff_gyroZ / buffersize;
        }
        i++;
        delay(2); //Needed so we don't get repeated measures
    }
}

void calibration() {


  
    accX_offset = -mean_accX / 8;
    accY_offset = -mean_accY / 8;
    accZ_offset = (16384 - mean_accZ) / 8;

    gyroX_offset = -mean_gyroX / 4;
    gyroY_offset = -mean_gyroY / 4;
    gyroZ_offset = -mean_gyroZ / 4;
    while (1) {
      
      static unsigned long cur_TimeoutCheck = 0;
      cur_TimeoutCheck = millis();

      if (cur_TimeoutCheck - pre_TimeoutCheck >= CALIBRATION_TIMEOUT ) {
      pre_TimeoutCheck = cur_TimeoutCheck;
      //ESP.restart();
      break;
    }
      
      //timeoutCheck 시간지나면 esp32 재시작 
        int ready = 0;
        accelgyro.setXAccelOffset(accX_offset);
        accelgyro.setYAccelOffset(accY_offset);
        accelgyro.setZAccelOffset(accZ_offset);

        accelgyro.setXGyroOffset(gyroX_offset);
        accelgyro.setYGyroOffset(gyroY_offset);
        accelgyro.setZGyroOffset(gyroZ_offset);

        meansensors();
        Serial.println("...");
        Serial.print(cur_TimeoutCheck / 1000);
        Serial.println("Seconds");
    
        if (abs(mean_accX) <= acel_deadzone) ready++;
        else accX_offset = accX_offset - mean_accX / acel_deadzone;

        if (abs(mean_accY) <= acel_deadzone) ready++;
        else accY_offset = accY_offset - mean_accY / acel_deadzone;

        if (abs(16384 - mean_accZ) <= acel_deadzone) ready++;
        else accZ_offset = accZ_offset + (16384 - mean_accZ) / acel_deadzone;

        if (abs(mean_gyroX) <= giro_deadzone) ready++;
        else gyroX_offset = gyroX_offset - mean_gyroX / (giro_deadzone + 1);

        if (abs(mean_gyroY) <= giro_deadzone) ready++;
        else gyroY_offset = gyroY_offset - mean_gyroY / (giro_deadzone + 1);

        if (abs(mean_gyroZ) <= giro_deadzone) ready++;
        else gyroZ_offset = gyroZ_offset - mean_gyroZ / (giro_deadzone + 1);

        if (ready == 6) break;
    }
}


//--------------------------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------CALIBRATION ROUTINE--------------------------------------------------------//


//-----------------------------------------------Cat.M1.Device Function-------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//

void serialPcInit(void)
{
  Serial.begin(115200);
}

void serialDeviceInit()
{
  Serial2.begin(WM_N400MSE_DEFAULT_BAUD_RATE,SERIAL_8N1,16,17);
}

void serialAtParserInit()
{
  m_parser.set_timeout(1000);
  m_parser.set_delimiter("\r");
}
void catm1DeviceInit()
{
  serialDeviceInit();
  serialAtParserInit();
}

// ----------------------------------------------------------------
// Functions: Cat.M1 Status
// ----------------------------------------------------------------

int8_t waitCatM1Ready()
{
  while (1)
  {
    if (m_parser.recv(F("RDY"))) {
      MYPRINTF("BG96 ready\r\n");
      return RET_OK;
    }
    else if (m_parser.send(F("AT")) && m_parser.recv(F(RESP_OK)))
    {
      MYPRINTF("BG96 already available\r\n");
      return RET_OK;
    }
  }
  return RET_NOK;
}


bool initStatus()
{

  if ( setEchoStatus_BG96(false) != RET_OK )
  {
    return false;
  }

  if ( getUsimStatus_BG96() != RET_OK )
  {
    return false;
  }

  if ( getNetworkStatus_BG96() != RET_OK )
  {
    return false;
  }

  return true;
}

int8_t setEchoStatus_BG96(bool onoff)
{
  if ( onoff == true )
  {
    if ( !(m_parser.send(F("ATE1")) && m_parser.recv(F(RESP_OK))) ) {
      LOGDEBUG("Echo On: Failed\r\n");
      return RET_NOK;
    }
    else
    {
      LOGDEBUG("Echo On: Success\r\n");
      return RET_OK;
    }

  }
  else if ( onoff == false )
  {
    if ( !(m_parser.send(F("ATE0")) && m_parser.recv(F(RESP_OK))) ) {
      LOGDEBUG("Echo Off: Failed\r\n");
      return RET_NOK;
    }
    else
    {
      LOGDEBUG("Echo Off: Success\r\n");
      return RET_OK;
    }
  }
}



int8_t getUsimStatus_BG96(void)
{
  char usim_stat[10], detail[10];
  char buf[40];

  if ( m_parser.send(F("AT+CPIN?")) &&
       m_parser.recv(F("+CPIN: READY\n")) &&
       m_parser.recv(F(RESP_OK)) ) {
    LOGDEBUG("USIM Status: READY\r\n");
    return RET_OK;
  }

  else if ( m_parser.send(F("AT+CPIN?")) &&
            m_parser.recv(F("+CPIN: %[^,],%[^\n]\n"), usim_stat, detail) &&
            m_parser.recv(F(RESP_OK)) ) {
    sprintf((char *)buf, "USIM Satatus: %s, %s", usim_stat, detail);
    LOGDEBUG(buf);
    return RET_NOK;
  }
}

int8_t checknSetApn_BG96(const char * apn) // Configure Parameters of a TCP/IP Context
{
  char resp_str[100];
  char buf[25];
  char buf1[25];

  uint16_t i = 0;
  char * search_pt;

  memset(resp_str, 0, sizeof(resp_str));

  LOGDEBUG("Checking APN...\r\n");


  m_parser.send(F("AT+QICSGP=1"));
  while (1)
  {
    m_parser.read(&resp_str[i++], 1);
    search_pt = strstr(resp_str, "OK\r\n");
    if (search_pt != 0)
    {
      break;
    }
  }

  search_pt = strstr(resp_str, apn);
  if (search_pt == 0)
  {
    sprintf((char *)buf, "Mismatched APN: %s\r\n", resp_str);
    sprintf((char *)buf1, "Storing APN %s...\r\n", apn);
    LOGDEBUG(buf);
    LOGDEBUG(buf1);

    if (!(m_parser.send("AT+QICSGP=1,%d,\"%s\",\"\",\"\",0", BG96_APN_PROTOCOL, apn) && m_parser.recv("OK")))
    {
      return RET_NOK; // failed
    }
  }
  LOGDEBUG("APN Check Done\r\n");

  return RET_OK;
}



int8_t getNetworkStatus_BG96(void)
{
  char mode[40], stat[40];
  char buf[40];

  if ( m_parser.send(F("AT+CEREG?")) &&
       m_parser.recv(F("+CEREG: %[^,],%[^\n]\n"), mode, stat) &&
       m_parser.recv(F(RESP_OK)) ) {

    if ( (atoi(mode) == 0) && (atoi(stat) == 1) ) {
      LOGDEBUG("Network Status: Attach\r\n");
      return RET_OK;
    }

    else if (( atoi(stat) != 1 )) {
      sprintf((char *)buf, "Network Status: %d, %d", atoi(mode), atoi(stat));
      LOGDEBUG(buf);
      return RET_NOK;
    }
  }
  return RET_NOK;
}

void getIMEIInfo_BG96(void)
{
  char m_imei[30];
  char buf[25];

  if ( (m_parser.send(F("AT*MINFO"))
        && m_parser.recv(F("*MINFO:%*[^,],%*[^,],%[^,],%*[^\n]\n"), m_imei)
        && m_parser.recv(F(RESP_OK))) ) {
    sprintf((char *)buf, "Module IME: %s\r\n", m_imei);
    LOGDEBUG(buf);
  }
}

int8_t getFirmwareVersion_BG96(char * version)
{
  int8_t ret = RET_NOK;

  if (m_parser.send("AT+QGMR") && m_parser.recv("%s\n", version) && m_parser.recv("OK"))
  {
    ret = RET_OK;
  }
  return ret;
}

int8_t getImeiNumber_BG96(char * imei)
{
  int8_t ret = RET_NOK;

  if (m_parser.send("AT+CGSN") && m_parser.recv("%s\n", imei) && m_parser.recv("OK"))
  {
    ret = RET_OK;
  }
  return ret;
}

// ----------------------------------------------------------------
// Functions: Cat.M1 DNS
// ----------------------------------------------------------------

int8_t getIpAddressByName_BG96(const char * name, char * ipstr)
{
  char buf2[50];
  bool ok;
  int  err, ipcount, dnsttl;

  int8_t ret = RET_NOK;

  ok = ( m_parser.send("AT+QIDNSGIP=1,\"%s\"", name)
         && m_parser.recv("OK")
         && m_parser.recv("+QIURC: \"dnsgip\",%d,%d,%d", &err, &ipcount, &dnsttl)
         && err == 0
         && ipcount > 0
       );

  if ( ok ) {
    m_parser.recv("+QIURC: \"dnsgip\",\"%[^\"]\"", ipstr);       //use the first DNS value
    for ( int i = 0; i < ipcount - 1; i++ )
      m_parser.recv("+QIURC: \"dnsgip\",\"%[^\"]\"", buf2);   //and discrard the rest  if >1

    ret = RET_OK;
  }
  return ret;
}

// ----------------------------------------------------------------
// Functions: Cat.M1 PDP context activate / deactivate
// ----------------------------------------------------------------

void setContextActivate_BG96(void)
{
  if ( (m_parser.send(F("AT+QIACT=1"))
        && m_parser.recv(F(RESP_OK))) ) {
    LOGDEBUG("PDP Context Activation: Success\r\n");
  }
}

int8_t setContextDeactivate_BG96(void) // Deactivate a PDP Context
{
  if ( (m_parser.send(F("AT+QIDEACT=1"))
        && m_parser.recv(F(RESP_OK))) ) {
    LOGDEBUG("PDP Context Deactivation: Success\r\n");
  }
}

int8_t getIpAddress_BG96(char * ipstr) // IPv4 or IPv6
{
  int8_t ret = RET_NOK;
  int id, state, type; // not used

  m_parser.send("AT+QIACT?");
  if (m_parser.recv("+QIACT: %d,%d,%d,\"%[^\"]\"", &id, &state, &type, ipstr)
      && m_parser.recv("OK")) {
    ret = RET_OK;
  }
  return ret;
}
// ----------------------------------------------------------------
// Functions: Cat.M1 MQTT Publish & Subscribe
// ----------------------------------------------------------------

int8_t openMqttBroker_BG96(char * url, int port)
{
  int8_t ret = RET_NOK;
  int id = 0;
  int result = 0;
  unsigned long lastOpenedTime = 0;         // last time you connected to the server, in milliseconds
  bool done = false;
  //Timer t;

  //t.start();
  lastOpenedTime = millis();

  if (m_parser.send("AT+QMTOPEN=%d,\"%s\",%d", id, url, port) && m_parser.recv("OK")) {
    do {
      done = (m_parser.recv("+QMTOPEN: %d,%d", &id, &result) && (result == 0));

      // MQTT Open: result code sample, refer to BG96_MQTT_Application_Note
      if (result == 1) {
        LOGDEBUG("AT+QMTOPEN result[1]: Wrong parameter");
      } else if (result == 2) {
        LOGDEBUG("AT+QMTOPEN result[2]: MQTT identifier is occupied");
      } else if (result == 3) {
        LOGDEBUG("AT+QMTOPEN result[3]: Failed to activate PDP");
      } else if (result == 4) {
        LOGDEBUG("AT+QMTOPEN result[4]: Failed to parse domain name");
      } else if (result == 5) {
        LOGDEBUG("AT+QMTOPEN result[5]: Network disconnection error");
      }
    } while (!done && (millis() - lastOpenedTime) < BG96_CONNECT_TIMEOUT);

    if (done) {
      ret = RET_OK;
    }
  }
  m_parser.flush();

  return ret;
}

int8_t connectMqttBroker_BG96(char * clientid, char * userid, char * password)
{
  int8_t ret = RET_NOK;
  int id = 0;
  int result = 0;
  int ret_code = 0;
  
  unsigned long lastConnectedTime = 0;         // last time you connected to the server, in milliseconds
  char buf[100];
  
  bool done = false;
  //Timer t;

  if ((userid != NULL) && (password != NULL)) {
    m_parser.send("AT+QMTCONN=%d,\"%s\",\"%s\",\"%s\"", id, clientid, userid, password);
  } else {
    m_parser.send("AT+QMTCONN=%d,\"%s\"", id, clientid);
  }

  //t.start();
  lastConnectedTime = millis();
  
  if (m_parser.recv("OK"))
  {
    do {
      done = (m_parser.recv("+QMTCONN: %d,%d,%d", &id, &result, &ret_code)
              && (result == 0) && (ret_code == 0));

      // MQTT Connect: result sample, refer to BG96_MQTT_Application_Note
      if (result == 1) {
        LOGDEBUG("AT+QMTCONN result[1]: Packet retransmission");
      } else if (result == 2) {
        LOGDEBUG("AT+QMTCONN result[2]: Failed to send packet");
      }

      // MQTT Connect: ret_code sample, refer to BG96_MQTT_Application_Note
      if (result == 1) {
        LOGDEBUG("AT+QMTCONN ret_code[1]: Connection Refused: Unacceptable Protocol Version");
      } else if (result == 2) {
        LOGDEBUG("AT+QMTCONN ret_code[2]: Connection Refused: Identifier Rejected");
      } else if (result == 3) {
        LOGDEBUG("AT+QMTCONN ret_code[3]: Connection Refused: Server Unavailable");
      } else if (result == 4) {
        LOGDEBUG("AT+QMTCONN ret_code[4]: Connection Refused: Bad User Name or Password");
      } else if (result == 5) {
        LOGDEBUG("AT+QMTCONN ret_code[5]: Connection Refused: Not Authorized");
      }
    } while (!done &&(millis() - lastConnectedTime) < BG96_CONNECT_TIMEOUT * 2);

    if (done) {
      ret = RET_OK;
    }
  }
  m_parser.flush();

  return ret;
}

int8_t closeMqttBroker_BG96(void)
{
  int8_t ret = RET_NOK;
  int id = 0;
  int result = 0;
  unsigned long lastClosedTime = 0;            // last time you connected to the server, in milliseconds

  bool done = false;
  //Timer t;

  //t.start();
  lastClosedTime = millis();
  
  if (m_parser.send("AT+QMTDISC=%d", id) && m_parser.recv("OK")) {
    do {
      done = (m_parser.recv("+QMTDISC: %d,%d", &id, &result));
    } while (!done && (millis() - lastClosedTime) < BG96_CONNECT_TIMEOUT * 2);

    if (done) {
      ret = RET_OK;
    }
  }
  m_parser.flush();

  return ret;
}


int8_t sendMqttPublishMessage_BG96(char * topic, int qos, int retain, char * msg, int len)
{
  int8_t ret = RET_NOK;
  int id = 0;
  int result = 0;
  int sent_msgid = 0;
  static int msgid = 0;
  
  unsigned long lastSentTime = 0;            // last time you connected to the server, in milliseconds
  char buf[100];
  
  bool done = false;
  //Timer t;

  if (qos != 0) {
    if (msgid < 0xffff)
      msgid++;
    else
      msgid = 0;
  }

  //t.start();
  lastSentTime = millis();
  
  m_parser.send("AT+QMTPUB=%d,%d,%d,%d,\"%s\"", id, qos ? msgid : 0, qos, retain, topic);

  if ( !done && m_parser.recv(">") )
    done = (m_parser.write(msg, len) <= 0) & m_parser.send("%c", MQTT_EOF);

  if (m_parser.recv("OK")) {
    do {
      done = (m_parser.recv("+QMTPUB: %d,%d,%d", &id, &sent_msgid, &result));
    } while (!done && (millis() - lastSentTime) < BG96_CONNECT_TIMEOUT * 2);

    if (done) {
      ret = RET_OK;
    }
  }
  m_parser.flush();

  return ret;
}

int8_t setMqttSubscribeTopic_BG96(char * topic, int msgid, int qos)
{
  int8_t ret = RET_NOK;
  int id = 0;
  int result = 0;

  int sent_msgid = 0;
  int qos_level = 0;
  unsigned long lastSetTime = 0;            // last time you connected to the server, in milliseconds

  bool done = false;
  //Timer t;

  m_parser.set_timeout(BG96_CONNECT_TIMEOUT);

  //t.start();
  lastSetTime = millis();
  
  if (m_parser.send("AT+QMTSUB=%d,%d,\"%s\",%d", id, msgid, topic, qos) && m_parser.recv("OK")) {
    do {
      done = (m_parser.recv("+QMTSUB: %d,%d,%d,%d", &id, &sent_msgid, &result, &qos_level));
    } while (!done && (millis() - lastSetTime) < BG96_CONNECT_TIMEOUT);

    if (done) {
      ret = RET_OK;
    }
  }
  m_parser.set_timeout(BG96_DEFAULT_TIMEOUT);
  m_parser.flush();

  return ret;
}

int8_t checkRecvMqttMessage_BG96(char * topic, int * msgid, char * msg)
{
  int8_t ret = RET_NOK;
  int id = 0;
  bool received = false;

  m_parser.set_timeout(1);
  received = m_parser.recv("+QMTRECV: %d,%d,\"%[^\"]\",\"%[^\"]\"", &id, msgid, topic, msg);
  m_parser.set_timeout(BG96_DEFAULT_TIMEOUT);

  if (received) ret = RET_OK;
  return ret;
}

void dumpMqttSubscribeTopic_BG96(char * topic, int msgid, int qos)
{
  char buf[100];
  sprintf((char *)buf, "[MQTT] Subscribe Topic: \"%s\", ID: %d, QoS: %d\r\n", topic, msgid, qos);
  MYPRINTF(buf);
}

void dumpMqttPublishMessage_BG96(char * topic, char * msg)
{
  char buf[100];
  sprintf((char *)buf, "[MQTT] Published Topic: \"%s\", Message: \"%s\"\r\n", topic, msg);
  MYPRINTF(buf);
}

// ----------------------------------------------------------------
// Functions: MQTT SSL/TLS enable
// ----------------------------------------------------------------

int8_t setMqttTlsEnable_BG96(bool enable) 
{
    int8_t ret = RET_NOK;
    
    int id = 0; // tcp connection id (0 - 6)
    int tls_ctxindex = 0; // ssl context index (0 - 5)
    
    if(m_parser.send("AT+QMTCFG=\"SSL\",%d,%d,%d", id, enable?1:0, tls_ctxindex) && m_parser.recv("OK")) {        
        ret = RET_OK;
    } else { 
        LOGDEBUG("MQTT SSL/TLS enable failed\r\n");
    }    
    return ret;
}

// ----------------------------------------------------------------
// Functions: Cat.M1 File system
// ----------------------------------------------------------------

int8_t saveFileToStorage_BG96(char * path, const char * buf, int len)
{
    int8_t ret = RET_NOK;
    int timeout_sec = 30;
    
    bool done = false;
    int upload_size = 0;
    char checksum[10] = {0, };    
    
    m_parser.set_timeout(BG96_WAIT_TIMEOUT);
        
    if(m_parser.send("AT+QFUPL=\"%s\",%d,%d", path, len, timeout_sec) && m_parser.recv("CONNECT")) {        
        done = m_parser.write(buf, len);    
        if(done) {
            if(m_parser.recv("+QFUPL: %d,%s\r\n", &upload_size, checksum) && m_parser.recv("OK")) {
                if(len == upload_size) {
                    LOGDEBUG("File saved: %s, %d, %s\r\n", path, upload_size, checksum);
                    ret = RET_OK;
                }
            }
        }
    }    
    m_parser.set_timeout(BG96_DEFAULT_TIMEOUT);        
    if(ret != RET_OK) {
        LOGDEBUG("Save a file to storage failed: %s\r\n", path);
    }
    m_parser.flush();
    
    delay(100);
    return ret;
}

int8_t eraseFileStorageAll_BG96(void)
{
    int8_t ret = RET_NOK;
    
    if(m_parser.send("AT+QFDEL=\"*\"") && m_parser.recv("OK")) {        
        ret = RET_OK;
    } else { 
        LOGDEBUG("Erase storage failed\r\n");
    }
    
    delay(100);
    return ret;
}

#define MAX_FILE_LIST           10
void dumpFileList_BG96(void)
{    
    char _buf[30] = {0, };
    int flen = {0, };
    int fcnt = 0;

    unsigned long lastSentTime = 0;
    
    bool done = false;
    //Timer t;
        
    //t.start();
    lastSentTime = millis();
    
    if(m_parser.send("AT+QFLST")) {        
        do {             
            if(m_parser.recv("+QFLST: \"%[^\"]\",%d\r\n", _buf, &flen)) {                
                LOGDEBUG("File[%d]: %s, %d\r\n", fcnt++, _buf, flen);
                memset(_buf, 0x00, sizeof(_buf));         
            }            
            else if(m_parser.recv("OK")) {
                done = true;
            }
        } while(!done && (millis() - lastSentTime) < BG96_WAIT_TIMEOUT);
    }
    m_parser.flush();
} 


// ----------------------------------------------------------------
// Functions: SSL/TLS config
// ----------------------------------------------------------------

int8_t setTlsCertificatePath_BG96(char * param, char * path)
{
    int8_t ret = RET_NOK;    
    int tls_ctxindex = 0;               // ssl context index (0 - 5)
    
    if(m_parser.send("AT+QSSLCFG=\"%s\",%d,\"%s\"", param, tls_ctxindex, path) && m_parser.recv("OK")) {        
        ret = RET_OK;
    } else { 
        LOGDEBUG("Set SSL/TLS certificate path failed: %s\r\n", param);
    }    
    return ret;
}

// 0: SSL3.0, 1: TLS1.0, 2: TLS1.1, 3: TLS1.2, 4: All
int8_t setTlsConfig_sslversion_BG96(int ver)
{
    int8_t ret = RET_NOK;    
    int tls_ctxindex = 0;                   // ssl context index (0 - 5)
    char param[] = "sslversion";            // ssl config paramter type       
    
    if(m_parser.send("AT+QSSLCFG=\"%s\",%d,%d", param, tls_ctxindex, ver) && m_parser.recv("OK")) {        
        ret = RET_OK;
    } else { 
        LOGDEBUG("Set SSL/TLS version failed: %d\r\n", ver);
    }    
    return ret;
}

int8_t setTlsConfig_ciphersuite_BG96(char * ciphersuite)    // refer to SSL manual
{
    int8_t ret = RET_NOK;
    int tls_ctxindex = 0;                   // ssl context index (0 - 5)
    char param[] = "ciphersuite";           // ssl config paramter type       
    
    if(m_parser.send("AT+QSSLCFG=\"%s\",%d,%s", param, tls_ctxindex, ciphersuite) && m_parser.recv("OK")) {        
        ret = RET_OK;
    } else { 
        LOGDEBUG("Set SSL/TLS Ciphersuite failed: %d\r\n", ciphersuite);
    }
    return ret;
}

int8_t setTlsConfig_seclevel_BG96(int seclevel)
{
    int8_t ret = RET_NOK;    
    int tls_ctxindex = 0;                   // ssl context index (0 - 5)
    char sslconfig[] = "seclevel";          // ssl config paramter type
    
    if(m_parser.send("AT+QSSLCFG=\"%s\",%d,%d", sslconfig, tls_ctxindex, seclevel) && m_parser.recv("OK")) {        
        ret = RET_OK;
    } else { 
        LOGDEBUG("Set SSL/TLS authentication mode failed: %d\r\n", seclevel);
    }
    return ret;
}
  
int8_t setTlsConfig_ignoreltime_BG96(bool enable)
{
    int8_t ret = RET_NOK;    
    int tls_ctxindex = 0;                   // ssl context index (0 - 5)
    char sslconfig[] = "ignorelocaltime";   // ssl config paramter type
    
    if(m_parser.send("AT+QSSLCFG=\"%s\",%d,%d", sslconfig, tls_ctxindex, enable?1:0) && m_parser.recv("OK")) {        
        ret = RET_OK;
    } else { 
        LOGDEBUG("Set SSL/TLS ignore validity check option failed: %s\r\n", sslconfig);
    }
    return ret;
}


// ----------------------------------------------------------------
// Functions: AWS IoT samples
// ----------------------------------------------------------------

int8_t aws_iot_connection_process(void)
{   
    static int8_t mqtt_state;

    switch(mqtt_state) {
        case MQTT_STATE_CONNECTED:        
            break;
            
        case MQTT_STATE_OPEN:
            if(openMqttBroker_BG96(AWS_IOT_MQTT_HOST, AWS_IOT_MQTT_PORT) == RET_OK) {
                MYPRINTF("[MQTT] Socket open success\r\n");
                mqtt_state = MQTT_STATE_CONNECT;
            } else {
                MYPRINTF("[MQTT] Socket open failed\r\n");
            }
            break;
            
        case MQTT_STATE_CONNECT:
            if(connectMqttBroker_BG96(AWS_IOT_MQTT_CLIENT_ID, NULL, NULL) == RET_OK) {
                MYPRINTF("[MQTT] Connected, ClientID: \"%s\"\r\n", AWS_IOT_MQTT_CLIENT_ID);
                mqtt_state = MQTT_STATE_CONNECTED;
            } else {
                MYPRINTF("[MQTT] Connect failed\r\n");
                mqtt_state = MQTT_STATE_DISCON;
            }
            break;
        
        case MQTT_STATE_DISCON:
            if(closeMqttBroker_BG96() == RET_OK) {
                MYPRINTF("[MQTT] Disconnected\r\n");                    
            }
            mqtt_state = MQTT_STATE_OPEN;
            break;
            
        default:                
            mqtt_state = MQTT_STATE_OPEN;
            break;
    }
    
    return mqtt_state;
}

void aws_flip() {
    flag_aws_publish = true;
}

// ----------------------------------------------------------------
// Functions: Cat.M1 GPS
// ----------------------------------------------------------------

int8_t setGpsOnOff_BG96(bool onoff)
{
  int8_t ret = RET_NOK;
  char _buf[15];
  char _buf1[30];
  char _buf2[30];

  sprintf((char *)_buf, "%s", onoff ? "AT+QGPS=2" : "AT+QGPSEND");

  if (m_parser.send(_buf) && m_parser.recv("OK")) {
    sprintf((char *)_buf1, "GPS Power: %s\r\n", onoff ? "On" : "Off");
    LOGDEBUG(_buf1);
    ret = RET_OK;
  } else {
    sprintf((char *)_buf2, "Set GPS Power %s failed\r\n", onoff ? "On" : "Off");
    LOGDEBUG(_buf2);
  }
  return ret;
}



int8_t getGpsLocation_BG96(gps_data data)
{
  int8_t ret = RET_NOK;
  char buf1[200];

  bool ok = false;
  //Timer t;

  // Structure init: GPS info
  //data->utc = data->lat = data->lon = data->hdop = data->altitude = data->cog = data->spkm = data->spkn = data->nsat = 0.0;
  //data->fix = 0;
  data.utc = data.lat = data.lon = data.hdop = data.altitude = data.cog = data.spkm = data.spkn = data.nsat = 0.0;
  data.fix = 0;
  //data->date = 0;
  //memset(&data->date, 0x00, 7);
  //strcpy(data->date, "");
  strcpy(data.date, "");

  // timer start
  //t.start();
  getLocationTime = millis();

    //while ( !ok && ( (millis() - getLocationTime) < BG96_CONNECT_TIMEOUT ) ) { 
    if ( !ok && (flag_aws_publish == true) ) {
    m_parser.flush();
    m_parser.send((char*)"AT+QGPSLOC=2"); // MS-based mode
    ok = m_parser.recv("+QGPSLOC: ");
    if (ok) {
      m_parser.recv("%s\r\n", buf1);
      sscanf(buf1, "%f,%f,%f,%f,%f,%d,%f,%f,%f,%6s,%d",
             &data.utc, &data.lat, &data.lon, &data.hdop,
             &data.altitude, &data.fix, &data.cog,
             &data.spkm, &data.spkn, &data.date, &data.nsat);

             //dtostrf(data.lat, 7, 5, latBuf);
             //dtostrf(data.lon, 8, 5, lonBuf);
             sprintf((char*)utcBuf, "%f", data.utc);
             sprintf((char*)latBuf, "%f", data.lat);
             sprintf((char*)lonBuf, "%f", data.lon);
             sprintf((char*)dateBuf, "%s", data.date);
             sprintf((char*)timestampBuf, "%.2f,%s", data.utc, data.date);

             Serial.print("utcBuf 값 : ");
             Serial.println(utcBuf);
             Serial.print("latBuf 값 : ");
             Serial.println(latBuf);
             Serial.print("lonBuf 값 : ");
             Serial.println(lonBuf);
             Serial.print("dateBuf 값 : ");
             Serial.println(dateBuf);
             Serial.print("timestampBuf 값 : ");
             Serial.println(timestampBuf);
             Serial.println("");
             Serial.print("buf1 값 : ");
             Serial.println((char*)buf1);


      ok = m_parser.recv("OK");
    }
  }

  if (ok == true) ret = RET_OK;
  


  return ret;
}

int bg96_ON(int pwrCheck)
{
  pwrCheck = 0;
 if (pwrCheck == 0) {
  digitalWrite(PWR_PIN, HIGH);
  digitalWrite(RST_PIN, HIGH);
  delay(300); // Setup Time : Greater than or equal to 30ms
  digitalWrite(PWR_PIN, LOW);
  digitalWrite(RST_PIN, LOW);
  delay(600); // Hold Time : Greater than or equal to 500ms
  digitalWrite(PWR_PIN, HIGH);
  delay(5000); // Release Time : Greater than or equal to 4800ms

  pwrCheck = 1;
  }
  return pwrCheck;
}

int bg96_DeviceRST(int rstCheck)
{
  rstCheck = 0;
  
 if (rstCheck == 0) {
  digitalWrite(RST_PIN, HIGH);
  digitalWrite(PWR_PIN, HIGH);
  delay(300); // Setup Time : Greater than or equal to 30ms
  digitalWrite(RST_PIN, LOW);
  digitalWrite(PWR_PIN, LOW);
  delay(400); // Hold Time : Greater than or equal to 500ms
  digitalWrite(RST_PIN, HIGH);
  delay(3000); // Release Time : Greater than or equal to 4800ms

  rstCheck = 1;
  }
  return rstCheck;
}

//efficiency algorithm//

void accelYpAlgorithm() //forward acceleration algorithm
{
  const float limitAccelY = 0.5;
  //static float pre_f_accelY = 0.0;
  //static int counter_AccelYp = 0;

  if ( (f_accelY >= limitAccelY) && (pre_f_accelY < limitAccelY) )
  {
    counter_AccelYp++;
    Serial.print("counter_AccelYp : ");
    Serial.println(counter_AccelYp);
  }

}

void fuelAlgorithm() //fuel consumption algorithm
{
  const int limitFuel = 8; // km/l
  //static int counter_Fuel = 0;
  const int limitFuel_inf = 14;
  static int counter_N = 1;
  //static float pre_instantFuelConsume = 0.0;

  averageFuelConsume = (float)((averageFuelConsume*(counter_N-1)+instantFuelConsume)/counter_N);

if (averageFuelConsume <= limitFuel_inf) {
  if ( (instantFuelConsume <= limitFuel) && (pre_instantFuelConsume > limitFuel) )
  {
    counter_Fuel++;
    Serial.print("counter_Fuel : ");
    Serial.println(counter_Fuel);
  }
  else if ( (instantFuelConsume >= limitFuel_inf) && (pre_instantFuelConsume < limitFuel_inf) )
  {
    counter_Fuel--;
    if (counter_Fuel <= 0 )
    {
      counter_Fuel = 0;
    Serial.print("counter_Fuel : ");
    Serial.println(counter_Fuel);
    }
  }
}

if (averageFuelConsume >= limitFuel_inf) 
{
    counter_Fuel = 0;
    Serial.print("counter_Fuel Zeroed ");
    Serial.println(counter_Fuel);
}
counter_N++;
}

void rpmAlgorithm() //enginespeed algorithm
{
  const int limitRpm = 2200;
  //static uint32_t pre_rpm = 0.0;
  //static int counter_Rpm = 0;

  if ( (rpm >= limitRpm) && (pre_rpm < limitRpm) )
  {
    counter_Rpm++;
    Serial.print("counter_Rpm : ");
    Serial.println(counter_Rpm);
  } 
}

void efficiencyMarkAlgorithm() //algorithm with fuel limit condition efficiency formulas
{
  //static float efficiencyMark = 0.0;
  const int limitFuel = 8;
  //const int limitFuel_inf = 12;

  if (averageFuelConsume < limitFuel)
  {
    efficiencyMark = (float)(5.0-(40*((float)counter_AccelYp)/runTime)-(30*((float)counter_Rpm)/runTime));
    Serial.print("efficiencyMark : ");
    Serial.print(efficiencyMark*10.0);
    Serial.println(" / 100 ");
  }
  else if (averageFuelConsume >= limitFuel)
  {
    efficiencyMark = (float)(10.0-(40*((float)counter_AccelYp)/runTime)-(180*((float)counter_Fuel)/runTime)-(30*((float)counter_Rpm)/runTime));
    Serial.print("efficiencyMark : ");
    Serial.print(efficiencyMark*10.0);
    Serial.println(" / 100 ");
  }
  Serial.print("averageFuelConsume : ");
  Serial.println(averageFuelConsume);
}


//safety algorithm//

void accelXAlgorithm() //lateral acceleration algorithm
{
  const float limitAccelXp = 0.4;
  const float limitAccelXm = -0.4;
  //static float pre_f_accelX = 0.0;
  //static int counter_AccelX = 0;

  if ( (f_accelX >= limitAccelXp) && (pre_f_accelX < limitAccelXp)) 
  {
    counter_AccelX++;
    Serial.print("counter_AccelX : ");
    Serial.println(counter_AccelX);
  }
  else if ( (f_accelX <= limitAccelXm) && (pre_f_accelX > limitAccelXm)) 
  {
    counter_AccelX++;
    Serial.print("counter_AccelX : ");
    Serial.println(counter_AccelX);
  }

}

void accelYmAlgorithm() //dangerous acceleration algorithm
{
  const float limitAccelYm = -0.4;
  //static float pre_f_accelY = 0;
  //static int counter_AccelYm = 0;

if ( (f_accelY <= limitAccelYm) && (pre_f_accelY > limitAccelYm) )
{
    counter_AccelYm++;
    Serial.print("counter_AccelYm : ");
    Serial.println(counter_AccelYm);
}
}

void accelYmAlgorithm2() // negative forward acceleration algorithm
{
  const float limitAccelYm = -0.3;
  //static float pre_f_accelY = 0.0;
  //static int counter_AccelYm2 = 0;

  if ( (f_accelY <= limitAccelYm) && (pre_f_accelY > limitAccelYm) )
  {
    counter_AccelYm2++;
    Serial.print("counter_AccelYm2 : ");
    Serial.println(counter_AccelYm2);
  }
}

void timeAlgorithm() //time algorithm
{
  const int limitTime = 7200; //2hr (7200sec)
  //static uint16_t pre_runTime = 0;
  //static int counter_Time = 0;
  
  if ( (runTime >= limitTime) && (pre_runTime < limitTime) )
  {
    counter_Time = 1;
    Serial.print("counter_Time : ");
    Serial.println(counter_Time);
  }
}

void kphAlgorithm() //vehicle speed algorithm
{
  const int limitKph = 51;
  //static int32_t pre_kph = 0;
  //static int counter_Kph = 0;

  if ( (kph >= limitKph) && (pre_kph < limitKph))
  {
    counter_Kph++;
    Serial.print("counter_Kph : ");
    Serial.println(counter_Kph);
  }
}

void safetyMarkAlgorithm() //algorithm with time condition formulas
{
  //static float safetyMark = 0.0;

  if (counter_Time == 0)
  {
    safetyMark = (float)(10.0-(200*((float)counter_AccelYm)/runTime)-(80*((float)counter_Kph)/runTime)-(50*((float)counter_AccelYm2)/runTime)-(30*((float)counter_AccelX)/runTime));
    Serial.print("safetyMark : ");
    Serial.print(safetyMark*10.0);
    Serial.println(" / 100 ");
  }
  else if (counter_Time > 0)
  {
    safetyMark = (float)(9.0-(200*((float)counter_AccelYm)/runTime)-(80*((float)counter_Kph)/runTime)-(50*((float)counter_AccelYm2)/runTime)-(30*((float)counter_AccelX)/runTime));
    Serial.print("safetyMark : ");
    Serial.print(safetyMark*10.0);
    Serial.println(" / 100 ");
  }
  Serial.print("Counter_Time : ");
  Serial.println(counter_Time);
}

void drivingMarkAlgorithm() //global algorithm
{
  //static float drivingMark = 10.0;
  
  drivingMark = ((safetyMark+efficiencyMark) / 2.0);
  char safetyScoreBuf[30] = "";

  if (drivingMark <= 2)
  {
    strcpy(safetyScoreBuf, "verybad");
    Serial.print("safetyScoreBuf : ");
    Serial.println(safetyScoreBuf);
    Serial.print("drivingMark : ");
    Serial.print(drivingMark*10.0);
    Serial.println(" / 100 ");
  }

  else if ( (drivingMark <= 4) && (drivingMark > 2) )
  {
    strcpy(safetyScoreBuf, "bad");
    Serial.print("safetyScoreBuf : ");
    Serial.println(safetyScoreBuf);
    Serial.print("drivingMark : ");
    Serial.print(drivingMark*10.0);
    Serial.println(" / 100 ");
  }

  else if ( (drivingMark <= 6) && (drivingMark > 4) )
  {
    strcpy(safetyScoreBuf, "normal");
    Serial.print("safetyScoreBuf : ");
    Serial.println(safetyScoreBuf);
    Serial.print("drivingMark : ");
    Serial.print(drivingMark*10.0);
    Serial.println(" / 100 ");
  }

  else if ( (drivingMark <= 8) && (drivingMark > 6) )
  {
    strcpy(safetyScoreBuf, "good");
    Serial.print("safetyScoreBuf : ");
    Serial.println(safetyScoreBuf);
    Serial.print("drivingMark : ");
    Serial.print(drivingMark*10.0);
    Serial.println(" / 100 ");
  }

  else if (drivingMark > 8)
  {
    strcpy(safetyScoreBuf, "verygood");
    Serial.print("safetyScoreBuf : ");
    Serial.println(safetyScoreBuf);
    Serial.print("drivingMark : ");
    Serial.print(drivingMark*10.0);
    Serial.println(" / 100 ");
  }

}

void averagedrivingMarkAlgorithm()
{
  //static float averagedrivingMark = 10.0;
  static int counter_N = 1;
  
  //if (drivingMark >= 0)
  averagedrivingMark = (float)((averagedrivingMark*(counter_N-1)+drivingMark)/counter_N);

  Serial.print("averagedrivingMark : ");
  Serial.print(averagedrivingMark*10.0);
  Serial.println(" / 100 ");

  counter_N++;
}

void pre_DataUpdate()
{
    pre_f_accelX = f_accelX;
    pre_f_accelY = f_accelY;
    pre_instantFuelConsume = instantFuelConsume;
    pre_rpm = rpm;
    pre_runTime = runTime;
    pre_kph = kph;

    Serial.print("pre_f_accelX = ");
    Serial.println(pre_f_accelX);
    Serial.print("pre_f_accelY = ");
    Serial.println(pre_f_accelY);
    Serial.print("pre_instantFuelConsume = ");
    Serial.println(pre_instantFuelConsume);
    Serial.print("pre_rpm = ");
    Serial.println(pre_rpm);
    Serial.print("pre_runTime = ");
    Serial.println(pre_runTime);
    Serial.print("pre_kph = ");
    Serial.println(pre_kph);
    
    Serial.println("pre_Data Updated");
}

void counterReset()
{
  counter_AccelYp = 0;
  counter_Fuel = 0;
  counter_Rpm = 0;
  efficiencyMark = 0.0;
  counter_AccelX = 0;
  counter_AccelYm = 0;
  counter_AccelYm2 = 0;
  counter_Time = 0;
  counter_Kph = 0;
  safetyMark = 0.0;
  drivingMark = 0.0;
  averagedrivingMark = 0.0;

  averageFuelConsume= 0.0;

  Serial.println("Counter Reset");
}

bool errorValueReset()
{
  errorvaluecheck = false;
   if ( (rpm == -1) || (engineLoad == -1) || (kph == -1) || (runTime == 65535) )
   {
    runTime = pre_runTime;
    rpm = 700;
    kph = 0;
    engineLoad = 19;
    oilTemp = 85;
    engineCoolantTemp = 85;
    relativePedalPos = 10;
    throttle = 13;
    relativeThrottle = 3;
    commandedThrottleActuator = 4;
    intakeAirTemp = 15;
    manifoldPressure = 32;
    mafRate = 2.01;
    //fuelLevel = 50;
    ctrlModVoltage = 14;
    distTravelWithMIL = 0;
    ambientAirTemp = 15;

    errorvaluecheck = true;
   }
   return errorvaluecheck;
}
