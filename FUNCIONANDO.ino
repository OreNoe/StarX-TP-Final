#include <SoftwareSerial.h>
#include <Wire.h>    // incluye libreria bus I2C
#include <MechaQMC5883.h> // incluye libreria para magnetometro QMC5883L
MechaQMC5883 qmc;


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define ORIENTAR_NORTE 1
#define MOVER_DIRECCION 2

#define OUTPUT_READABLE_YAWPITCHROLL
const int stepZ = 4;

const int dirZ  = 7;

const int StepY = 3;
const int DirY = 6;

int maquinaEstados;

#define INTERRUPT_PIN  18 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  qmc.init();     // inicializa objeto


  Serial.begin(9600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  Serial.println("1er Paso: Espere a que el telescopio se posicione correctamente de manera automatica");
  Serial.println("2do Paso: Ingrese su latitud y longitud en esta pagina: https://efemeridesastronomicas.dyndns.org");
  Serial.println("3er Paso: Busque en la opcion Herramientas ---> Cuerpos celestes, el cuerpo que desee ver");
  Serial.println("4to Paso: Busque en la hoja que le aparece los grados que le indica la opcion de Azimut (0°N) y Altura");
  Serial.println("5to Paso: Copie y pegue las coordenadas indicadas en el programa en el orden mencionado");


  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);


  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
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
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  pinMode(StepY, OUTPUT);
  pinMode(DirY, OUTPUT);

  Serial.println("Orientando al norte");
  maquinaEstados = ORIENTAR_NORTE;

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================









void loop() {

  /*// if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    float yaw = ypr[0] * 180 / M_PI;
    float pitch = ypr[1] * 180 / M_PI;
    float roll = ypr[2] * 180 / M_PI;
    Serial.print("ypr\t");
    Serial.print(yaw);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.println(roll);
    digitalWrite(DirY, HIGH);*/

  int x, y, z;
  float acimut, geografico; // variables para acimut magnetico y geografico    // declinacion desde pagina: http://www.magnetic-declination.com/
  qmc.read(&x, &y, &z, &acimut); // funcion para leer valores y asignar a variables

  geografico = acimut - 9.13;  // acimut geografico como suma del magnetico y declinacion
  if (geografico < 0) { // si es un valor negativo
    geografico = geografico + 360;  // suma 360 y vuelve a asignar a variable
  }
  switch (maquinaEstados) {
    case RECIBIR_AZIMUTH:
      Serial.println("Ingresa del azimuth tal cual la pagina");
      if (Serial.available() > 0) {
      // 
      char firstChar = Serial.read();
      break;
    case ORIENTAR_NORTE:
      Serial.print("Acimut geografico: ");
      Serial.print(geografico, 2);  Serial.println("°");
      if (geografico > 358 || geografico < 2) {
        Serial.println("Orientado realizado exitosamente");
        maquinaEstados = MOVER_DIRECCION;
      }
      break;

    case MOVER_DIRECCION:

      break;
  }
  //moverANorte ();
  // apuntadoTelescopio(yaw, pitch, roll);


}
//}


/*void moverANorte (int geografico) {
  /*float acimut, geografico, y; // variables para acimut magnetico y geografico
  float declinacion = -9.32;    // declinacion desde pagina: http://www.magnetic-declination.com/
  geografico = acimut + declinacion;  // acimut geografico como suma del magnetico y declinacion
  if (geografico < 0) {    // si es un valor negativo
  geografico = geografico + 360;  // suma 360 y vuelve a asignar a variable
  }
  Serial.print(geografico);*/
/*if (geografico < 358 && geografico > 2) {
  Serial.print("Moviendose");
  for (int x = 0; x < 200; x++) { // loop for 200 steps
    digitalWrite(StepY, HIGH);
    delayMicroseconds(500);
    digitalWrite(StepY, LOW);
    delayMicroseconds(500);
  }
  } else {
  Serial.print("Parado, calibracion completada");



  }*/
/*if (y < 358 && y > 2) {

  Serial.print("Moviendose");
  for (int x = 0; x < 200; x++) { // loop for 200 steps
    digitalWrite(StepY, HIGH);
    delayMicroseconds(500);
    digitalWrite(StepY, LOW);
    delayMicroseconds(500);

  } else {
    Serial.print("parado");
  }
  }*/
//}
/*
  void apuntadoTelescopio(float yaw, float pitch, float roll) {

    int idx;
    int grados, mins, segs, num;
    float grDecimAz;//Grados en decimales
    float altura;
    //Serial.println("Ingrese sus coordenadas azimuth. Comenzando con un "*" (asterizco) y terminando con un "#"(numeral)");
    Serial.print("Ejemplo: *220°40'20");
    Serial.println("4");

    if (Serial.available() > 0) {
      // *220°40'20"4#
      char firstChar = Serial.read();

      if (firstChar == '*') {
        // 220°40'20"4#
        String message = Serial.readStringUntil('#');
        // 220°40'20"4

        idx = message.indexOf('°');
        if (idx > -1) {
          grados = message.substring(0, idx).toInt();
          message = message.substring(idx + 1);

          // 10,250,8
          idx = message.indexOf("'");
          if (idx > -1) {
            mins = message.substring(0, idx).toInt();
            message = message.substring(idx + 1);

            // 250,8
            idx = message.indexOf('"');
            if (idx > -1) {
              segs = message.substring(0, idx).toInt();
              message = message.substring(idx + 1);

              // 8
              num = message.toInt();

              float grDecimAz = grados + 1 / 60 * mins + 1 / 3600 * segs;
              Serial.println(grDecimAz);
              if (grDecimAz != 0)
              {
                yaw = 0;
                Serial.println("MOVIENDOSE");
                if (yaw == grDecimAz)
                {
                  Serial.println("Parado");
                }
              } else {
                Serial.println("Objeto no identificado");
              }



            } else {
              Serial.println("Mensaje erroneo");
            }
          } else {
            Serial.println("Mensaje erroneo");
          }
        } else {
          Serial.println("Mensaje erroneo");
        }
      }
    }*/
//HACER LO MISMO QUE ARRIBA PERO CON LOS GRADOS PARA LA ALTURA
//  }
