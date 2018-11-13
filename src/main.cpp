#include <Arduino.h>
#include <WiFi.h>

#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>

#include "Wire.h"

#define __PGMSPACE_H_ 1 // stop compile errors of redefined typedefs and defines with ESP32-Arduino

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define DEBUG 0
#define LED_PIN 13

const char* ssid = "O-Zone";
const char* password = "B00mSt!ck";
IPAddress server(192,168,1,85);       // Set the rosserial socket ROSCORE SERVER IP address
const uint16_t serverPort = 11411;    // Set the rosserial socket server port

boolean connected = false;

//These are the object handlers of TF message and broadcaster 
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

//Creating handlers of Node, IMU message, quaternion and ROS publisher.
ros::NodeHandle nh;

//Creating orientation message header 
geometry_msgs::Vector3 orient;

//Creating ROS publisher object for IMU orientation
ros::Publisher imu_pub("imu_data", &orient);
ros::Time current_time = nh.now();

//The frame_id helps to visualise the Transform data of IMU w.r.t this link
char frameid[] = "/base_link";
char child[] = "/imu_frame";

//Creating a MPU6050 handle which can be used for MPU 9250
MPU6050 mpu(0x69);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//#define INTERRUPT_PIN_MPU 19
//volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

volatile int mpuDataCounter = 0;
int mpuDataCounterPrev = 0;
/*
void IRAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}
*/
void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case SYSTEM_EVENT_STA_GOT_IP:
            // When connected set
            Serial.print("WiFi connected! IP address: ");
            Serial.println(WiFi.localIP());
      
            nh.getHardware()->setConnection(server, serverPort);
            nh.initNode();
            broadcaster.init(nh);

            nh.advertise(imu_pub);

            Serial.printf("Wifi Event running on core %d", (int)xPortGetCoreID());
            Serial.println();

            connected = true;
            digitalWrite(LED_PIN, true);

            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            Serial.println("WiFi lost connection.  Attempting to reconnect...");
			WiFi.begin(ssid, password);
            connected = false;
            digitalWrite(LED_PIN, false);
            break;
    }
}

void setupWiFi() {
    WiFi.mode(WIFI_OFF);
    //register event handler
    WiFi.onEvent(WiFiEvent);
    WiFi.disconnect(true);
    WiFi.enableAP(false);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    uint8_t count = 0;
    wl_status_t status = WiFi.status();
    while (status != WL_CONNECTED && count < 40) {
         
         switch (status) {
            case WL_NO_SSID_AVAIL:
                Serial.println("No SSID");
                break;
            case WL_CONNECT_FAILED:
                Serial.println("Connection failed");
                break;
            case WL_CONNECTED:
                break;
        }
        delay(500);
        count++;
        status = WiFi.status();
    }
}

void sensorTask( void * pvParameters ) {
    delay(100);

    String taskMessage = "sensorTask running on core ";
    taskMessage = taskMessage + xPortGetCoreID();

    Serial.println(taskMessage);
    Wire.begin();
    Wire.setClock(400000);

    delay(1000);

    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for someone else's test chip
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_MPU), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
         // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    while (true) {
        if (connected) {
            // if programming failed, don't try to do anything
            if (!dmpReady) {
                Serial.println("dmpNotReady");
                delay(1000);
            }

            // wait for MPU interrupt or extra packet(s) available
            //while (!mpuInterrupt && fifoCount < packetSize) {
            //      ;
            //}

            // reset interrupt flag and get INT_STATUS byte
            //mpuInterrupt = false;
            mpuIntStatus = mpu.getIntStatus();

            // get current FIFO count
            fifoCount = mpu.getFIFOCount();

            // check for overflow (this should never happen unless our code is too inefficient)
            if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                // reset so we can continue cleanly
                mpu.resetFIFO();
                Serial.println(F("FIFO overflow!"));

            // otherwise, check for DMP data ready interrupt (this should happen frequently)
            } else if (mpuIntStatus & 0x01) {
                // wait for correct available data length, should be a VERY short wait
                while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

                // read a packet from FIFO
                mpu.getFIFOBytes(fifoBuffer, packetSize);
                
                // track FIFO count here in case there is > 1 packet available
                // (this lets us immediately read more without waiting for an interrupt)
                fifoCount -= packetSize;

                // display quaternion values in easy matrix form: w x y z
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                
                mpuDataCounter++;                
            }
        } else { // not connected
            vTaskDelay(10); // wait and feed the watchdog timer
        }
    }
}

void setup() {
    Serial.begin(115200);

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    //Connect to the WiFi network
    setupWiFi();

    xTaskCreatePinnedToCore(
        sensorTask, /* Function to implement the task */
        "coreTask", /* Name of the task */
        10000,      /* Stack size in words */
        NULL,       /* Task input parameter */
        20,         /* Priority of the task */
        NULL,       /* Task handle. */
        1);   /* Core where the task should run */
}

void loop() {
    if (nh.connected()) {
        current_time = nh.now();
        if (mpuDataCounter != mpuDataCounterPrev) {
            //Assigning YAW,PITCH,ROLL to vector message and publishing the values
            orient.x = ypr[0] * 180 / M_PI;
            orient.y = ypr[1] * 180 / M_PI;
            orient.z = ypr[2] * 180 / M_PI;

            imu_pub.publish(&orient);

            //Assigning values to TF header and publish the transform
            t.header.frame_id = frameid;
            t.child_frame_id = child;
            t.transform.translation.x = 1.0;
            t.transform.rotation.x = q.x;
            t.transform.rotation.y = q.y; 
            t.transform.rotation.z = q.z; 
            t.transform.rotation.w = q.w;  
            t.header.stamp = nh.now();
            broadcaster.sendTransform(t);
        }
    } else { // not connected
        vTaskDelay(10); // wait and feed the watchdog timer.
    }

    nh.spinOnce();
}

