#include "Arduino.h"
#include <wavelib.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#define MPL115A2_DEFAULT_ADDRESS (0x60)
#define MPL115A2_REGISTER_PRESSURE_MSB (0x00) /**< 10-bit Pressure ADC output value MSB **/
#define MPL115A2_REGISTER_PRESSURE_LSB (0x01) /**< 10-bit Pressure ADC output value LSB **/
#define MPL115A2_REGISTER_TEMP_MSB (0x02) /**< 10-bit Temperature ADC output value MSB **/
#define MPL115A2_REGISTER_TEMP_LSB (0x03) /**< 10-bit Temperature ADC output value LSB **/
#define MPL115A2_REGISTER_A0_COEFF_MSB (0x04)  /**< a0 coefficient MSB **/
#define MPL115A2_REGISTER_A0_COEFF_LSB (0x05)  /**< a0 coefficient LSB **/
#define MPL115A2_REGISTER_B1_COEFF_MSB (0x06)  /**< b1 coefficient MSB **/
#define MPL115A2_REGISTER_B1_COEFF_LSB (0x07)  /**< b1 coefficient LSB **/
#define MPL115A2_REGISTER_B2_COEFF_MSB (0x08)  /**< b2 coefficient MSB **/
#define MPL115A2_REGISTER_B2_COEFF_LSB (0x09)  /**< b2 coefficient LSB **/
#define MPL115A2_REGISTER_C12_COEFF_MSB (0x0A) /**< c12 coefficient MSB **/
#define MPL115A2_REGISTER_C12_COEFF_LSB (0x0B) /**< c12 coefficient LSB **/
#define MPL115A2_REGISTER_STARTCONVERSION (0x12) /**< Start Pressure and Temperature Conversion **/

#define checkPin 2
#define a1 3 // 700 upper  v5-1
#define a2 4 // 1200 upper v5-e2
#define RPin1 8 // red
#define RPin2 9 // green
#define RPin3 10 // green
#define RPin4 11 // green
#define GPin1 7 // adafruit
#define GPin2 15 // green
#define GPin3 16 // green
#define GPin4 17 // green
#define NUMPIXELS 2 //# of pixels

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, GPin1, NEO_GRBW + NEO_KHZ800);

TwoWire *_wire;
uint8_t _i2caddr;
uint16_t pressure, temp;

int count;
int buf1[20];int buf2[20];
double *inp,*out,*diff;
int N = 16; int J=4; int i,j; int x1, x2, x3, x4; 
double d4_a1[20]; double d4_a2[20];
double d4_re_a1[2], d4_re1_a1[5], d4_re2_a1[10], d4_re3_a1[20];
double d4_re_a2[2], d4_re1_a2[5], d4_re2_a2[10], d4_re3_a2[20];
double amp = 0.7071067811;

int _mpl115a2_a0;
int _mpl115a2_b1;
int _mpl115a2_b2;
int pressureComp;

long interval = 500; 
long previousMillis;
unsigned long currentMillis; 

uint8_t i2cread(TwoWire *_wire) {
  uint8_t x;
  x = _wire->read();
  return x;
}

void i2cwrite(TwoWire *_wire, uint8_t x) {
  _wire->write((uint8_t)x);
}

void readCoefficients() {
  int16_t a0coeff;
  int16_t b1coeff;
  int16_t b2coeff;
  //int16_t c12coeff;

  _wire->beginTransmission(_i2caddr);
  i2cwrite(_wire, (uint8_t)MPL115A2_REGISTER_A0_COEFF_MSB);
  _wire->endTransmission();
  
  _wire->requestFrom(_i2caddr, (uint8_t)8);
  a0coeff = (((uint16_t)i2cread(_wire) << 8) | i2cread(_wire));
  b1coeff = (((uint16_t)i2cread(_wire) << 8) | i2cread(_wire));
  b2coeff = (((uint16_t)i2cread(_wire) << 8) | i2cread(_wire));

  _mpl115a2_a0 = (int)a0coeff / 8;
  _mpl115a2_b1 = (int)b1coeff / 8192;
  _mpl115a2_b2 = (int)b2coeff / 16384;
}

float time() {
  return float( micros() ) * 1e-6;
}

void delay_(int ms) {
  while (count != ms) {
    delayMicroseconds(1000);
    count++;
  }
  count = 0;
}

void pressure_sensor() {

  _wire->beginTransmission(_i2caddr);
  i2cwrite(_wire, (uint8_t)MPL115A2_REGISTER_STARTCONVERSION);
  i2cwrite(_wire, (uint8_t)0x00);
  _wire->endTransmission();
  delayMicroseconds(1500);
 
  _wire->beginTransmission(_i2caddr);
  i2cwrite(_wire, (uint8_t)MPL115A2_REGISTER_PRESSURE_MSB); // Register
  _wire->endTransmission();

  _wire->requestFrom(_i2caddr, (uint8_t)4);
  pressure = (((uint16_t)i2cread(_wire) << 8) | i2cread(_wire)) >> 6;
  temp = (((uint16_t)i2cread(_wire) << 8) | i2cread(_wire)) >> 6;

  pressureComp = _mpl115a2_a0 + (_mpl115a2_b1) * pressure + _mpl115a2_b2 * temp;
  //Serial.print(pressureComp); Serial.print(',');
}

void wavelet_transform1() {
  
  wave_object obj;
  wt_object wt; 

  char *name = "haar"; 
  obj = wave_init(name);

  inp = (double*)malloc(sizeof(double)* N);
  out = (double*)malloc(sizeof(double)* N);

  for (i = 0; i < N; ++i) {
    inp[i] = buf1[i];
  }

  wt = wt_init(obj, "dwt", N, J);// Initialize the wavelet transform object
  setDWTExtension(wt, "sym");// Options are "per" and "sym". Symmetric is the default option
  setWTConv(wt, "direct");
  dwt(wt, inp); //perform DWT for sensor 1

//unreconstructed dwt 
//  for (i = (N/16); i < (N/16)*2; ++i) {
//    Serial.print(wt->output[i]); 
//  }

//saving d4
  for (j = 0; j < (N/16) ; ++j) {
    d4_a1[j] = wt->output[j+N/16];
    //Serial.println(d4_a1[j]);
  }
  
  wave_free(obj);
  wt_free(wt);
  free(inp);
  free(out);
}

void wavelet_transform2() {
  
  wave_object obj;
  wt_object wt; 

  char *name = "haar"; 
  obj = wave_init(name);

  inp = (double*)malloc(sizeof(double)* N);
  out = (double*)malloc(sizeof(double)* N);

  for (i = 0; i < N; ++i) {
    inp[i] = buf2[i];
  }

  wt = wt_init(obj, "dwt", N, J);// Initialize the wavelet transform object
  setDWTExtension(wt, "sym");// Options are "per" and "sym". Symmetric is the default option
  setWTConv(wt, "direct");
  dwt(wt, inp); //perform DWT for sensor 1

//unreconstructed dwt 
//  for (i = (N/16); i < (N/16)*2; ++i) {
//    Serial.print(wt->output[i]);
//  }

//saving d4
  for (j = 0; j < (N/16) ; ++j) {
    d4_a2[j] = wt->output[j+N/16];
    //Serial.print(d4_a2[j]);
  }
  
  wave_free(obj);
  wt_free(wt);
  free(inp);
  free(out);
}

void reconstruction1() {
  for (x1 = 0; x1 < N/16; ++x1) {
    d4_re_a1[x1 * 2] = d4_a1[x1]*amp;
    d4_re_a1[x1 * 2 + 1] = d4_a1[x1]*(-amp);
  }

  for (x2 = 0; x2 < N/8; ++x2) {
    d4_re1_a1[x2 * 2] = d4_re_a1[x2]*amp;
    d4_re1_a1[x2 * 2 + 1] = d4_re_a1[x2]*amp;
  }

  for (x3 = 0; x3 < N/4; ++x3) {
    d4_re2_a1[x3 * 2] = d4_re1_a1[x3]*amp;
    d4_re2_a1[x3 * 2 + 1] = d4_re1_a1[x3]*amp;
  }

  for (x4 = 0; x4 < N/2; ++x4) {
    d4_re3_a1[x4 * 2] = d4_re2_a1[x4]*amp;
    d4_re3_a1[x4 * 2 + 1] = d4_re2_a1[x4]*amp;
  }

  //reconstructed data output
//  for (i = 0; i< N; i++){
//    Serial.print(buf1[i]-700); Serial.print(',');
//    Serial.print(d4_re3[i]); //Serial.print(',');
//    Serial.print('\n');
//  }
}

void reconstruction2() {
  for (x1 = 0; x1 < N/16; ++x1) {
    d4_re_a2[x1 * 2] = d4_a2[x1]*amp;
    d4_re_a2[x1 * 2 + 1] = d4_a2[x1]*(-amp);
  }

  for (x2 = 0; x2 < N/8; ++x2) {
    d4_re1_a2[x2 * 2] = d4_re_a2[x2]*amp;
    d4_re1_a2[x2 * 2 + 1] = d4_re_a2[x2]*amp;
  }

  for (x3 = 0; x3 < N/4; ++x3) {
    d4_re2_a2[x3 * 2] = d4_re1_a2[x3]*amp;
    d4_re2_a2[x3 * 2 + 1] = d4_re1_a2[x3]*amp;
  }

  for (x4 = 0; x4 < N/2; ++x4) {
    d4_re3_a2[x4 * 2] = d4_re2_a2[x4]*amp;
    d4_re3_a2[x4 * 2 + 1] = d4_re2_a2[x4]*amp;
  }

  //reconstructed data output
//  for (i = 0; i< N; i++){
//    Serial.print(buf1[i]-700); Serial.print(',');
//    Serial.print(d4_re3[i]); //Serial.print(',');
//    Serial.print('\n');
//  }
}
 
void data_save1(){
  //digitalWrite(a1,HIGH);
  for(int a=0 ; a < N ; a++){
    readCoefficients();
    pressure_sensor();
    buf1[a] = pressureComp;
  }

//unreconstructed result
//  for(int b=0; b<N; b++){
//    Serial.print(buf1[b]-700);
//    Serial.print(',');
//  }
}

void data_save2(){
  //digitalWrite(a1,HIGH);
  for(int a=0 ; a < N ; a++){
    readCoefficients();
    pressure_sensor();
    buf2[a] = pressureComp;
  }

//unreconstructed result
//  for(int b=0; b<N; b++){
//    Serial.print(buf2[b]-1200);
//    Serial.print(',');
//  }
}

void setup() {
  Serial.begin(115200);
  _i2caddr = MPL115A2_DEFAULT_ADDRESS;
  _wire = &Wire;
  _wire->begin();
  pinMode(checkPin, OUTPUT);
  pinMode(a1,OUTPUT);
  pinMode(a2,OUTPUT);
  pinMode(RPin1, OUTPUT);
  pinMode(RPin2, OUTPUT);
  pinMode(RPin3, OUTPUT);
  pinMode(RPin4, OUTPUT);
  //pinMode(GPin1, OUTPUT);
  pinMode(GPin2, OUTPUT);
  pinMode(GPin3, OUTPUT);
  pinMode(GPin4, OUTPUT);
  digitalWrite(a1,LOW);
  digitalWrite(a2,LOW);
  digitalWrite(RPin1,LOW);
  digitalWrite(RPin2,LOW);
  digitalWrite(RPin3,LOW);
  digitalWrite(RPin4,LOW);
  //digitalWrite(GPin1,HIGH);
  digitalWrite(GPin2,HIGH);
  digitalWrite(GPin3,HIGH);
  digitalWrite(GPin4,HIGH);

  pixels.begin(); // This initializes the NeoPixel library.
  pixels.setBrightness(250); // LED 밝기 : 255가 최대 0이 최소 입니다.
  pixels.show();

}

void loop() {
  
  digitalWrite(a1,HIGH);
  data_save1();
  wavelet_transform1();
  reconstruction1();
  digitalWrite(a1,LOW);
  
  digitalWrite(a2,HIGH);
  data_save2();
  wavelet_transform2();
  reconstruction2();
  digitalWrite(a2,LOW);

//unreconstructed result
//  for (i = 0; i< N/16; i++){
//    Serial.print(d4_a1[i]); Serial.print(',');
//    Serial.print(d4_a2[i]);
//    Serial.print('\n');
//  }

//reconstructed result
  for (i = 0; i< N; i++){
    //Serial.print(x); Serial.print(',');
    Serial.print(buf1[i]-650); Serial.print(',');
    Serial.print(d4_re3_a1[i]); Serial.print(',');
    Serial.print(buf2[i]-650); Serial.print(',');
    Serial.print(d4_re3_a2[i]); Serial.print('\n');
    
    if(d4_re3_a1[i]>8 || d4_re3_a2[i]>8){
      previousMillis = millis();
      digitalWrite(RPin1, HIGH); 
      digitalWrite(RPin2, HIGH); 
      digitalWrite(RPin3, HIGH);
      digitalWrite(RPin4, HIGH);
      pixels.setPixelColor(0, pixels.Color(0,0,0,0)); // 0번 LED 빨간색 ON
      pixels.setPixelColor(1, pixels.Color(0,0,0,0)); // 1번 LED 빨간색 ON
      pixels.show(); 


//      digitalWrite(GPin1, LOW); 
//      digitalWrite(GPin2, LOW); 
//      digitalWrite(GPin3, LOW);
//      digitalWrite(GPin4, LOW);
    }
  }
  
  currentMillis = millis();
  if(currentMillis - previousMillis > interval){
  digitalWrite(RPin1,LOW);
  digitalWrite(RPin2,LOW);
  digitalWrite(RPin3,LOW);
  digitalWrite(RPin4,LOW);

  pixels.setPixelColor(0, pixels.Color(0,255,0,255)); // 0번 LED 빨간색 ON
  pixels.setPixelColor(1, pixels.Color(0,0,0,255)); // 1번 LED 빨간색 ON
  pixels.show(); 
//  digitalWrite(GPin1,HIGH);
//  digitalWrite(GPin2,HIGH);
//  digitalWrite(GPin3,HIGH);
//  digitalWrite(GPin4,HIGH);
  }
}
