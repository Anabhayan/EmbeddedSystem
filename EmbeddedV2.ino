/*
Written by Anabhayan September 7th/8th
Edited by Adhithya September 8th


Core code structure (PID) adapted from CurioRES https://youtu.be/dTGITLnYAY0
*/

template <int order>
class LowPass{
  private:
    float a[order];
    float b[order+1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order+1]; // Raw values
    float y[order+1]; // Filtered values

  public:  
    LowPass(float f0, float fs, bool adaptive){
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history. 
      omega0 = 6.28318530718*f0;
      dt = 1.0/fs;
      adapt = adaptive;
      tn1 = -dt;
      for(int k = 0; k < order+1; k++){
        x[k] = 0;
        y[k] = 0;        
      }
      setCoef();
    }

    void setCoef(){
      if(adapt){
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
      }
      
      float alpha = omega0*dt;
      if(order==1){
        a[0] = -(alpha - 2.0)/(alpha+2.0);
        b[0] = alpha/(alpha+2.0);
        b[1] = alpha/(alpha+2.0);        
      }
      if(order==2){
        float alphaSq = alpha*alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];
        b[0] = alphaSq/D;
        b[1] = 2*b[0];
        b[2] = b[0];
        a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
        a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;      
      }
    }

    float filt(float xn){
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if(adapt){
        setCoef(); // Update coefficients if necessary      
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for(int k = 0; k < order; k++){
        y[0] += a[k]*y[k+1] + b[k]*x[k];
      }
      y[0] += b[order]*x[order];

      // Save the historical values
      for(int k = order; k > 0; k--){
        y[k] = y[k-1];
        x[k] = x[k-1];
      }
  
      // Return the filtered value    
      return y[0];
    }
};

LowPass<2> lp(1,1e3,true);


#include <NewPing.h>

// USER INPUT
float SET_LEVEL = 200.0;  // Desired water level in millimeters


// SET INTIAL VALUES - dependent on shape of container 
const float USS_POS = 10.19;  // Height of USS from the bottom of the beaker (without water)
const float POS_ini_100mL = 2;  //Height of intial 100mL from bottom
const float POS_next_100mL = 1.5;  //Height of the next 100mL from intial 100mL mark
const float MAX_CAP = 500;  // max capacity of container in mL

float SET_POINT ;

void find_height() {
SET_LEVEL = constrain(SET_LEVEL, 0, MAX_CAP);

// height of water column for given volume
if( SET_LEVEL <= 100 ){
  SET_POINT = (SET_LEVEL / 100.0) * POS_ini_100mL; // 
}
else if( SET_LEVEL > 100 ){
  SET_POINT = POS_ini_100mL + ((SET_LEVEL - 100.0) / 100.0) * POS_next_100mL; // 
}

}



// PID constants
const float KP = 1.0;  // Proportional gain
const float KI = 0.0;  // Integral gain
const float KD = 0.0;  // Derivative gain
const float K = 100;    // Scaling factor

#define PWM_OUT 5
const int TRIGGER_PIN = 9;
const int ECHO_PIN = 10;
const int MAX_DISTANCE = 100;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

float distance ;
long prevMicros = 0;
float prevError = 0;
float integral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(PWM_OUT, OUTPUT);
}

void loop() {
  long currentMicros = micros();
  float deltaTime = (float)(currentMicros - prevMicros) / 1.0e6;
  prevMicros = currentMicros;

  // distance = USS_POS - measureDistance();
  distance = USS_POS - measureDistance_LPF();

  float error = SET_POINT - distance;
  
  integral += error * deltaTime;
  float derivative = (error - prevError) / deltaTime;
  
  float output = KP * error + KI * integral + KD * derivative;
  
  int pwmOutput = int(output * K);
  pwmOutput = constrain(pwmOutput, 0, 255);

  prevError = error;

  controlPump(pwmOutput);

  Serial.print(SET_POINT);
  Serial.print(" ");
  Serial.print(distance);
  Serial.println();
}

float measureDistance() {
  float cm = (sonar.ping() / 2.0) * 0.0343;  // Convert ping time to centimeters
  return cm;
}

float measureDistance_LPF() {
  float cm = lp.filt((sonar.ping() / 2.0) * 0.0343);  // Convert ping time to centimeters
  return cm;
}
void controlPump(int pwm) {
  analogWrite(PWM_OUT, pwm);
}

// pwm tester
/*
#define PWM_OUT 5

void setup() {
  // put your setup code here, to run once:

}
void loop() {
  // put your main code here, to run repeatedly:
    analogWrite(PWM_OUT, 200);

}
*/

