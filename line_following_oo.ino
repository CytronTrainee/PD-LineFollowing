//Line Following Robot using PD Controller (Digital)


//Initialization
int sensor[5] = {0, 0, 0, 0, 0};     //initialization for LSS05
int initial_motor_speed = 40;        //can change the initial speed that you prefer
int maxSpeed = 80;                 //if the motor is too fast, decrease the max speed and vice versa
float Kp = 0;       //will be change using potentiometer
float Kd = 0;       //will be change using potettiometer
float pinPD = 0;    //pin Analog 0 for the input of the potentiometer
float P=0, D=0, PD_value=0;
float error=0, previous_error=0;

//Pin declaration for SHIELD-2AMOTOR  
int RightEn = 5;
int RightDir = 4;
int LeftEn = 6;
int LeftDir = 7;

void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);

void setup()
{
  Serial.begin(9600);       //Enable Serial Communications
  pinMode (pinPD, INPUT);   //Pin Setup for potentiometer
  
  //LSS05 Auto-Calibrating Line Sensor Pin Setup
  pinMode(sensor[0],INPUT);
  pinMode(sensor[1],INPUT);
  pinMode(sensor[2],INPUT);
  pinMode(sensor[3],INPUT);
  pinMode(sensor[4],INPUT);

  //Motor Driver Pin Setup
  pinMode(RightEn,OUTPUT);
  pinMode(RightDir,OUTPUT);
  pinMode(LeftEn,OUTPUT);
  pinMode(LeftDir,OUTPUT);
  digitalWrite(RightDir,HIGH);    //according to your connection from motor to the motor driver
  digitalWrite(LeftDir,LOW);      //according to your connection from motor to the motor driver
}

void loop()
{
    read_sensor_values();
    calculate_pid();
    motor_control();
}

void read_sensor_values()
{
  sensor[0] = digitalRead(A1);
  sensor[1] = digitalRead(A2);
  sensor[2] = digitalRead(A3);
  sensor[3] = digitalRead(A4);
  sensor[4] = digitalRead(A5); 
  
  if((sensor[0]==0) && (sensor[1]==0) && (sensor[2]==1) && (sensor[3]==0) && (sensor[4]==0))
  error = 0;
  else if((sensor[0]==0) && (sensor[1]==0) && (sensor[2]==1) && (sensor[3]==1) && (sensor[4]==0))
  error = 1;
  else if((sensor[0]==0) && (sensor[1]==0) && (sensor[2]==0) && (sensor[3]==1) && (sensor[4]==0))
  error = 2;
  else if((sensor[0]==0) && (sensor[1]==0) && (sensor[2]==0) && (sensor[3]==1) && (sensor[4]==1))
  error = 4;
  else if((sensor[0]==0) && (sensor[1]==0) && (sensor[2]==0) && (sensor[3]==0) && (sensor[4]==1))
  error = 3;
  else if((sensor[0]==0) && (sensor[1]==1) && (sensor[2]==1) && (sensor[3]==0) && (sensor[4]==0))
  error = -1;
  else if((sensor[0]==0) && (sensor[1]==1) && (sensor[2]==0) && (sensor[3]==0) && (sensor[4]==0))
  error = -2;
  else if((sensor[0]==1) && (sensor[1]==1) && (sensor[2]==0) && (sensor[3]==0) && (sensor[4]==0))
  error = -4;
  else if((sensor[0]==1) && (sensor[1]==0) && (sensor[2]==0) && (sensor[3]==0) && (sensor[4]==0))
  error = -3;
  else if((sensor[0]==0) && (sensor[1]==0) && (sensor[2]==0) && (sensor[3]==0) && (sensor[4]==0))
    if(error==5) error = 5;
    else error = -5;
}

void calculate_pid()
{  
    P = error;
    D = error - previous_error;
    
    //Kp = analogRead(pinPD);
    //Kp = Kp/100;
    //Serial.println(Kp);
    
    //Kd = analogRead(pinPD);
    //Kd = Kd/100;
    //Serial.println(Kd);
    
    PD_value = (Kp*P) + (Kd*D);
    
    previous_error = error;
}

void motor_control()
{
    // Calculating the effective motor speed:
    int left_motor_speed = initial_motor_speed + PD_value;
    int right_motor_speed = initial_motor_speed - PD_value;
    
    // The motor speed should not exceed the max PWM value
    constrain(left_motor_speed,0,maxSpeed);
    constrain(right_motor_speed,0,maxSpeed);
    
    right_motor_speed = right_motor_speed + 5 ; //this line is needed if your motor didn't have same speed
    
    //open the Serial Monitor to see the speed of each motor
    Serial.print ("right = ");
    Serial.print(right_motor_speed); 
    Serial.print("\t");
    Serial.print("left = "); 
    Serial.println(left_motor_speed);  
    
    analogWrite(LeftEn,left_motor_speed);   //Left Motor Speed
    analogWrite(RightEn,right_motor_speed);  //Right Motor Speed
}

