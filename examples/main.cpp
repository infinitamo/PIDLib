
#include <iostream>
#include "PIDLib.hpp"

PIDLib::MeasurementType myMeas{};
PIDLib::SetupType myPIDSetup{};

PIDLib::PID myPID{};

int main()
{
    // PID Setup 
    double gainP = 1.0;
    double gainI = 0.1;
    double gainD = 0.5;
    double outputLowerLimit = -10.0;
    double outputUpperLimit = +10.0;

    myPIDSetup.gainP = gainP;
    myPIDSetup.gainI = gainI;
    myPIDSetup.gainD = gainD;
    myPIDSetup.outputLowerLimit = outputLowerLimit;
    myPIDSetup.outputUpperLimit = outputUpperLimit;

    // PID Init
    myPID.begin(myPIDSetup);

    // PID Run
    double setpoint = 5.0;

    double output{};

    myPID.setpoint(setpoint);
    
    myMeas.time_usec = 1000;
    myMeas.value     = 3.14;
    output = myPID.run(myMeas); // test 1st pass

    myMeas.time_usec = 2000;
    myMeas.value     = 3.5;
    output = myPID.run(myMeas); // test 2nd pass

    return 0;
}