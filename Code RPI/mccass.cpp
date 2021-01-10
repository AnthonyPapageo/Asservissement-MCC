#include <iostream>
#include <wiringPi.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
using namespace std;

void generalSignalHandler(int);
void clearingGpios(void);
int16_t getAngle(void);
void set_TxHigh(void);
void set_TxLow(void);
void myDelay(long int);
void Pid(int pin, float ku, float tu, int cons, int err, int* lasterr, float det);
void verify(int angle, int command, int cons);
void changeRotation();

bool rotPositive;


int oldAngle = 0;
int cycleCounter = 1;

int main() {
	cout << "mccass.cpp Started. PID = " << getpid() <<endl;

    struct sigaction signalAction;
    sigemptyset(&signalAction.sa_mask);
    signalAction.sa_flags = 0;
    signalAction.sa_handler = &generalSignalHandler;
    if (sigaction(SIGINT, &signalAction, NULL) == -1){
        std::cerr << "Impossible d'intercepter SIGINT !" << std::endl;
    }

    const int GPIO18 = 18; 	//PWM
    const int GPIO3 = 3; 	//Rot+ (Positif Pour Nous)
    const int GPIO4 = 4; 	//Rot- (Negatif Pour Nous)
    const int GPIO17 = 17;	//RxPi
    const int GPIO27 = 27;	//TxPi
    int GPIO20 = 20;	//Debug mesure
    const int GPIO21 = 21; //Interrupt Pour dire au stm que le code commence
    //long arg = strtol(argv[1], NULL, 10);
    //int arg = 42;

    //std::cout << "argvc : " << arg << std::endl;

    int cons = 180;	//Consigne Pid
    float Ku = 50; //Proportional
    float tu = 0.065;
    int err = 0; //Current Error
    int* lasterr; //Last Error
    float dt = 0.005;

    *lasterr = 0;
    //int16_t angle = 0;

	wiringPiSetupGpio();

	pinMode(GPIO18, PWM_OUTPUT);
	pinMode(GPIO3, OUTPUT);
	pinMode(GPIO4, OUTPUT);
	pinMode(GPIO21, OUTPUT);
	//pinMode(GPIODebug, OUTPUT);
	pinMode(GPIO17, INPUT);
	pinMode(GPIO27, OUTPUT);

	pwmSetClock(64000); //19.2Mhz/64000 //500Ha
	//digitalWrite(GPIO2, 1);

	digitalWrite(GPIO21, 0);

	if(cons>180){
		cons = -360+cons;
	}else if(cons < -180){
		cons = 360 + cons;
	}

	if(cons<0){
		digitalWrite(GPIO3, 1); //Rot+
		digitalWrite(GPIO4, 0); //Rot-
		rotPositive = false;
	}else if(cons > 0){
		digitalWrite(GPIO3, 0);
		digitalWrite(GPIO4, 1);
		rotPositive = true;
	}

	digitalWrite(GPIO27, 1); //TxPi HIGH
	pwmWrite(GPIO18, 1000);
	digitalWrite(GPIO21, 1);
	while(1){
		//digitalWrite(GPIODebug, 0);
		Pid(GPIO18, Ku, tu, cons, err, lasterr, dt);
		//digitalWrite(GPIODebug, 1);
		usleep(dt * 1000000);

		//std::cout << "Angle1 = " << getAngle() << std::endl;

		/*std::cout << "Rot+:0 (ON); Rot-:1 (OFF)" << std::endl;
		digitalWrite(GPIO3, 0);
		digitalWrite(GPIO4, 1);
		usleep(10000000);
		//std::cout << "Angle2 = " << getAngle() << std::endl;

		std::cout << "Rot+:1 (OFF); Rot-:0 (ON)" << std::endl;
		digitalWrite(GPIO3, 1);
		digitalWrite(GPIO4, 0);
		usleep(10000000);*/
	}

	return 0;
}



void Pid(int pin, float ku, float tu, int cons, int err, int* lasterr, float det)
{
	FILE *fp;

	int16_t angle;
	float ti;
	float td;
	float kp;

	float command = 0.;
	float integral =0;
	float derivative = 0;

	//Ziegler Classic Pid
	kp =0.6*ku;
	ti = 0.5 * tu;
	td = 0.125 * tu;

	//Some Overshoot
	/*kp=0.33 * ku;
	ti = 0.4 * tu;
	td = 0.33 * tu;*/

	integral += err;//*det;
	derivative = (err-*lasterr); // /det;

	angle = getAngle();
	err = cons - angle;

	//command = (ku * err); //Pour les tests ziglor, ki et kd = 0
	command = kp * ( err + (1/ti) * (integral * det) + (td * derivative/det));
	if(command<0)
		command = -command;

	if(command>1024.)
		command = 1024.;


	std::cout << "Commande, angle : " << command <<" " <<angle << std::endl;

	if(angle>cons && (rotPositive)){
		changeRotation();
	}

	else if(angle<cons && (rotPositive==false)){
		changeRotation();
	}


	/*if(cons-angle < 0){
		changeRotation();
	}*/


	pwmWrite(pin, (int)command);

	*lasterr = err;

	verify(angle, (int)command, cons);

    fp = fopen("./data.csv","a");
    fprintf(fp, "%d ; %d; %f ; \n",cycleCounter, angle, command);
    fclose(fp);

	oldAngle = angle;

	cycleCounter++;
}

void changeRotation()
{
    const int GPIO3 = 3; 	//Rot+ (Positif Pour Nous)
    const int GPIO4 = 4; 	//Rot- (Negatif Pour Nous)

    if(!(rotPositive)){
    	digitalWrite(GPIO3, 0);
    	digitalWrite(GPIO4, 1); //Actif en LOW
    	rotPositive = true;
    }else{
    	digitalWrite(GPIO3, 1);
		digitalWrite(GPIO4, 0); //Actif en LOW
		rotPositive = false;
    }
    std::cout << "senspos : " << rotPositive << std::endl;
}

void set_TxHigh()
{
	digitalWrite(27, 1);
}

void set_TxLow()
{
	digitalWrite(27, 0);
}

int read_Rx()
{
	return digitalRead(17);
}

int16_t getAngle()
{
	int16_t angle = 0;

	set_TxLow(); 		//TxLow Starting Comm
	while(read_Rx()==1){} //Wait for STM32

	for(int i=0; i<16; i++){
		angle = angle << 1;
		set_TxHigh();
		myDelay(10);
		set_TxLow();
		angle |= read_Rx();
		myDelay(10);
	}

	set_TxHigh();

	return angle;
}

void clearingGpios()
{
	digitalWrite(18, 0);
	digitalWrite(3, 1);
	digitalWrite(4, 1);
	set_TxHigh();

}

void generalSignalHandler(int signal)
{
    if (signal == SIGINT)
    {

        std::cout << "\nCtrl+c : Clearing GPIOS..." << std::endl;
        //std::cout << "Angle final = " << getAngle() << std::endl;
        clearingGpios();

        std::cout << "Done, program ending..." << std::endl;
        exit(0);
    }
    else
        std::cerr << "Signal non-géré" << std::endl;
}

void myDelay(long int usec){
	struct timespec request, remain;

	request.tv_sec = 0;
	request.tv_nsec = usec *1000;

	while(clock_nanosleep(CLOCK_MONOTONIC, 0,&request, &remain)){
		request = remain;
	}

}

void verify(int angle, int command, int cons)
{
    static int counter = 0;
    static int stuckCounter = 0;
    static int directionCounter = 0;

    if(command != 0 && !(angle > cons-5 && angle < cons+5))
    {
        if(oldAngle == angle)
            stuckCounter++;
        if(stuckCounter >30)
        {
            stuckCounter = 0;
            std::cerr << "MOTOR IS BLOCKED BIG PROBLEM" << std::endl;
        }
    }

    if(!(rotPositive) && oldAngle < angle )
    {
        directionCounter++;
    }
    else if(rotPositive && oldAngle > angle)
    {
        directionCounter++;
    }
    else
    {
        directionCounter = 0;
    }

    if(directionCounter > 10)
    {
        directionCounter = 0;
        std::cerr << "MOTOR IS TURNING IN THE WRONG DIRECTION BIG PROBLEM" << std::endl;
    }

    counter++;

    //std::cout << "counter, directioncounter, stuckcounter : " << counter << "|"<< directionCounter<< "|"<< stuckCounter << std::endl;
    if(counter > 100)
    {
        directionCounter = 0;
        stuckCounter = 0;
        counter = 0;
    }
}
