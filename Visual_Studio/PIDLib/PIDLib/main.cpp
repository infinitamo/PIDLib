
#include <iostream>
#include "PIDLib.hpp"

PIDLib_MeasurementType myMeas{};
PIDLib_PIDSetupType myPIDSetup{};

PIDLib_PID myPID{};

int main()
{
    // PID Setup 
    float gainP = 1.0f;
    float gainI = 0.1f;
    float gainD = 0.5f;
    float outputLowerLimit = -10.0f;
    float outputUpperLimit = +10.0f;

    myPIDSetup.gainP = gainP;
    myPIDSetup.gainI = gainI;
    myPIDSetup.gainD = gainD;
    myPIDSetup.outputLowerLimit = outputLowerLimit;
    myPIDSetup.outputUpperLimit = outputUpperLimit;

    // PID Init
    myPID.init(myPIDSetup);

    // PID Run
    float setpoint = 5.0f;

    float output{};

    myPID.setpoint(setpoint);
    
    myMeas.time_usec = 1000;
    myMeas.value     = 3.14f;
    output = myPID.run(myMeas); // test 1st pass

    myMeas.time_usec = 2000;
    myMeas.value     = 3.5f;
    output = myPID.run(myMeas); // test 2nd pass

    return 0;
}