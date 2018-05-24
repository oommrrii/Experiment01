#include <MatrixMath.h>

//#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Assign 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *Motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *Motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *Motor4 = AFMS.getMotor(4);

// defines Sonar pins numbers
const int frontTrigPin = 6;
const int frontEchoPin = 7;
const int rearTrigPin = 8;
const int rearEchoPin = 9;

// defines  Sonar variables
float Distance[2] = {0.1,0.1}; //0 for front distance; 1 for rear distance

float Previous_distance = 0, dt = 1/5.5; //algorithm works at 5.5 Hz

int count = 0, Time;

// Variables for Estimation
#define N  (3)

// Motion model
float A[N][N] = {{1,dt,0},{0,1,dt},{0,0,1}};
float At[N][N], Bt[N];
float B[N] = {0,0,dt};
float u = 0.01;//9.8 * 0.33/1.20; //Projection of gravity [m/s^2]
float U[1] = {u};
float X_[N], temp_vec[N], temp_vec3[N];
float X[N] = {0.05,0,u}; //Initial conditions

// Measurement model
float H[1][N] = {{1,0,0}};
float Ht[N][1];
float D[N] = {1,0,0};
float n = 0.05; //sensor noise [m]
float Z[1] = {0}; //Initialize measurement

// Error covariance
float P[N][N] = {{0.05,0,0},{0,0.05,0},{0,0,0.1}}; //Initial conditions [m]
float Pn[1] = {0.10}; //[m]
float P_[N][N], temp_mat[N][N], temp_mat2[N][N];
float R[1] = {0.05}; //sensor error covariance[m]

// Kalman gain
float K[N][1];
float temp_vec2[1][N], temp_scal[1], temp_scal2[1];

// Unit metrix
float I[N][N] = {{1,0,0},{0,1,0},{0,0,1}};


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Setuping the vehicle...");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  Motor1->setSpeed(150);
  Motor1->run(FORWARD);
  Motor1->run(RELEASE);

  Motor2->setSpeed(150);
  Motor2->run(FORWARD);
  Motor2->run(RELEASE);

  Motor3->setSpeed(150);
  Motor3->run(FORWARD);
  Motor3->run(RELEASE);

  Motor4->setSpeed(150);
  Motor4->run(FORWARD);
  Motor4->run(RELEASE);

  pinMode(frontTrigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(frontEchoPin, INPUT); // Sets the echoPin as an Input
  pinMode(rearTrigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(rearEchoPin, INPUT); // Sets the echoPin as an Input


  //Set pin 13 as output LED
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("Waiting for Flag to start algorithm");

}

// ########################## functions #######################

int drive(int l, int r){
    
  char lDirection, rDirection;
  if (l >= 0){
    lDirection = FORWARD;
  }
  else{
    lDirection = BACKWARD;
  }
  if (r >= 0){
    rDirection = FORWARD;
  }
  else{
    rDirection = BACKWARD;
  }

  if (abs(l) > 255 || abs(r) > 255){
  Serial.println("Error: Drive input value is greater than 255");
  //return 1;
    if (abs(l) > 255)
    l=255;
    if (abs(r) > 255)
    r=255;
  }
  
  Motor1->setSpeed(abs(l));
  Motor1->run(lDirection);
  
  Motor2->setSpeed(abs(l));
  Motor2->run(lDirection);
  
  Motor3->setSpeed(abs(r));
  Motor3->run(rDirection);
  
  Motor4->setSpeed(abs(r));
  Motor4->run(rDirection);
  return 0;
}


float * GetSonar(){

// defines  Sonar variables
long frontDuration;
long rearDuration;
static float Distance_meassured[2]; //0 for front meassured distance; 1 for rear meassured distance

// ############# Checking front distance #########

// Clears the trigPin
digitalWrite(frontTrigPin, LOW);
delayMicroseconds(2);

// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(frontTrigPin, HIGH);
delayMicroseconds(10);
digitalWrite(frontTrigPin, LOW);

// Reads the echoPin, returns the sound wave travel time in microseconds
frontDuration= pulseIn(frontEchoPin, HIGH);

// Sound velocity is 0.034 cm/usec
// Calculating the distance
Distance_meassured[0]= 0.01*frontDuration*0.034/2; //Distance in meters

// Prints the distance on the Serial Monitor
//Serial.print("Front Distance: ");
//Serial.println(Distance_meassured[0]);

// ############# Checking rear distance #########

// Clears the trigPin
digitalWrite(rearTrigPin, LOW);
delayMicroseconds(2);

// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(rearTrigPin, HIGH);
delayMicroseconds(10);
digitalWrite(rearTrigPin, LOW);

// Reads the echoPin, returns the sound wave travel time in microseconds
rearDuration = pulseIn(rearEchoPin, HIGH);

// Sound velocity is 0.034 cm/usec
// Calculating the distance
Distance_meassured[1]= 0.01*rearDuration*0.034/2; //Distance in meters

// Prints the distance on the Serial Monitor
//Serial.print("Rear Distance: ");
//Serial.println(Distance_meassured[1]);

// ############# Wait 60 miliseconds between samplings ##############
delay(60);

return Distance_meassured;

}


void Smooth_Sonar(){
// ############# Filtering the sonar measurements #########

// Declaring array variable for sonar meassurement function
float *Distance_meassured = GetSonar();

// Low pass filter coeficient. (alpha = dt/tau). dt = Sonar sampling rate is 40 Hz. tau = above frequencies will be filtered.
float alpha=0.5;

Distance[0]= Distance[0] + alpha*(*Distance_meassured - Distance[0]);
Distance[1]= Distance[1] + alpha*(*(Distance_meassured+1) - Distance[1]);

/*
 * Ploting 4 signals to the plotter. To check the filter's performance.
Serial.print(*Distance_meassured);
Serial.print(',');
Serial.print(Distance[0]);
Serial.print(',');
Serial.print(*(Distance_meassured+1));
Serial.print(',');
Serial.println(Distance[1]);
*/

/*
Serial.println("Front , Rear Distances: ");
Serial.print(Distance[0]);
Serial.print(" , ");
Serial.println(Distance[1]);
*/
}


void Sonar_reading_check(){
  Smooth_Sonar();
  Serial.println("Front , Rear Distances: ");
  Serial.print(Distance[0]);
  Serial.print(",");
  Serial.println(Distance[1]);
}


void loop() {
/*
if (count==100){
  Serial.println("Algorithm performs 100 times during the following number of micro seconds:");
  Serial.println(micros()-Time);
  float deltaT = micros()-Time;
  deltaT = deltaT/1000000;
  Serial.println("Algorithm performs in the following Hz:");
  Serial.println(100/deltaT);
  while(1);
}
*/
// ########################### Algorithm #############################

// Prediction
Matrix.Multiply((float*)A, (float*)X, N, N, 1, (float*)temp_vec);

//Matrix.Print((float*)temp_vec, N, 1, "AX");
//delay(5000);
Matrix.Multiply((float*)B, (float*)U, N, 1, 1, (float*)temp_vec3);

//Matrix.Print((float*)temp_vec3, N, 1, "BU");
//delay(5000);
  Matrix.Add((float*) temp_vec, (float*) temp_vec3, N, 1, (float*) X_);

//Matrix.Print((float*)X_, N, 1, "X_");
//delay(5000);





Matrix.Multiply((float*)A, (float*)P, N, N, N, (float*)P_);

//Matrix.Print((float*)P_, N, N, "P_");
//delay(5000); 
Matrix.Transpose((float*)A, N, N, (float*)At);

//Matrix.Print((float*)At, N, N, "At");
//delay(5000); 
Matrix.Multiply((float*)P_, (float*)At, N, N, N, (float*)temp_mat);

//Matrix.Print((float*)temp_mat, N, N, "temp_mat");
//delay(5000); 
Matrix.Multiply((float*)B, (float*)Pn, N, 1, 1, (float*)temp_vec);

//Matrix.Print((float*)temp_vec, N, 1, "temp_vec");
//delay(5000); 
Matrix.Transpose((float*)B, N, 1, (float*)Bt);

//Matrix.Print((float*)Bt, N, 1, "Bt");
//delay(5000); 
Matrix.Multiply((float*)temp_vec, (float*)Bt, N, 1, N, (float*)temp_mat2);

//Matrix.Print((float*)temp_mat2, N, N, "temp_mat2");
//delay(5000); 
  Matrix.Add((float*) temp_mat, (float*) temp_mat2, N, N, (float*) P_);
  
//Matrix.Print((float*)P_, N, N, "P_");
//delay(5000);  






// Kalman gain
Matrix.Transpose((float*)H, 1, N, (float*)Ht);

//Matrix.Print((float*)Ht, N, 1, "Ht");
//delay(5000);
Matrix.Multiply((float*)P_, (float*)Ht, N, N, 1, (float*)temp_vec);

//Matrix.Print((float*)temp_vec, N, 1, "temp_vec");
//delay(5000);

Matrix.Multiply((float*)H, (float*)P_, 1, N, N, (float*)temp_vec2);
Matrix.Multiply((float*)temp_vec2, (float*)Ht, 1, N, 1, (float*)temp_scal);
Matrix.Add((float*) temp_scal, (float*) R, 1, 1, (float*) temp_scal2); //?????????????????
  
  Matrix.Invert((float*)temp_scal2, 1);
Matrix.Multiply((float*)temp_vec, (float*)temp_scal2, N, 1, 1, (float*)K);

//Matrix.Print((float*)K, N, 1, "K");
//delay(5000);






// Declaring array variable for sonar meassurement function and measuring
float *Distance_meassured = GetSonar(); // obtain distance values
Z[0] = *Distance_meassured; //substituting measurment to the correct member of the vector






// Update
Matrix.Multiply((float*)H, (float*)X_, 1, N, 1, (float*)temp_scal);
Matrix.Subtract((float*) Z, (float*) temp_scal, 1, 1, (float*) temp_scal2);
Matrix.Multiply((float*)K, (float*)temp_scal2, N, 1, 1, (float*)temp_vec);
Matrix.Add((float*) X_, (float*) temp_vec, N, 1, (float*) X);

//Matrix.Print((float*)X, N, 1, "X");
//delay(5000);



Matrix.Multiply((float*)K, (float*)H, N, 1, N, (float*)temp_mat);
Matrix.Subtract((float*) I, (float*) temp_mat, N, N, (float*) temp_mat2);
Matrix.Multiply((float*)temp_mat2, (float*)P_, N, N, N, (float*)P);

//Matrix.Print((float*)P, N, N, "P");
//delay(5000);


// Ploting Estimated state
/*
Serial.println("\n Updated state:");
Matrix.Print((float*)X, N, 1, "X");
Serial.println("\n Updated state covariance:");
Matrix.Print((float*)P, N, N, "P");
*/
  
// Ploting measurement
float Velocity = (*Distance_meassured - Previous_distance)/dt; // Numerical calculation for velocity
Previous_distance = *Distance_meassured; // Save distance for next iteration

Serial.print(*Distance_meassured);
Serial.print(",");
Serial.print(X[0]);
Serial.print(",");
Serial.print(Velocity);
Serial.print(",");
Serial.println(X[1]);
/*
if (count==0)
Time = micros();
*/
count++;

}

