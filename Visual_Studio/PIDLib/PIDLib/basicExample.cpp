
#include <iostream>
#include "PIDLib.hpp"

void setup();
void loop();

PIDLib::MeasurementType myMeas{};
PIDLib::SetupType       myPIDSetup{};
PIDLib::PID             myPID{};

int main()
{
    // Crude example of using PIDLib in setup/loop
    setup();
    loop();

    return 0;
}

void setup()
{
    // PID Setup 
    myPIDSetup.PonX = PIDLib::ONX_MODE::ERROR;
    myPIDSetup.IonX = PIDLib::ONX_MODE::ERROR;
    myPIDSetup.DonX = PIDLib::ONX_MODE::MEASR;
    myPIDSetup.gainP = 1.0;
    myPIDSetup.gainI = 0.1;
    myPIDSetup.gainD = 0.5;
    myPIDSetup.outputLowerLimit = -10.0;
    myPIDSetup.outputUpperLimit = +10.0;

    // PID Init
    myPID.begin(myPIDSetup);
}

void loop()
{
    double setpoint = 5.0;
    double output{};

    myPID.setpoint(setpoint);
    
    myMeas.time_usec = 1000;
    myMeas.value     = 3.14;
    output = myPID.run(myMeas); // test 1st pass

    myPIDSetup.gainP = 2.0; // ability to change gains
    myPIDSetup.gainI = 0.2; // ability to change gains
    myPIDSetup.gainD = 0.1; // ability to change gains 

    myPID.config(myPIDSetup);  // config different settings

    myMeas.time_usec = 2000;
    myMeas.value     = 3.5;
    output = myPID.run(myMeas); // test 2nd pass
}