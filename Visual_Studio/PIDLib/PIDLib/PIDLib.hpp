#ifndef PID_LIB_HPP
#define PID_LIB_HPP

//
// Measurement Type Struct
//
struct PIDLib_MeasurementType
{
    PIDLib_MeasurementType() :
        time_sec( 0.0f ),
        value( 0.0f )
    {};

    float time_sec; // time of measurement in seconds
    float value;    // value of the measurement
};

//
// PID Controller Setup Struct
//
struct PIDLib_PIDSetupType
{
    PIDLib_PIDSetupType() : 
        gainP( 0.0f ), 
        gainI( 0.0f ), 
        gainD( 0.0f ), 
        outputLowerLimit( 0.0f ),
        outputUpperLimit( 0.0f )
    {};

    float gainP;             // proportional gain
    float gainI;             // integral     gain
    float gainD;             // derivative   gain
    float outputLowerLimit;  // desired lower limit of the output
    float outputUpperLimit;  // desired upper limit of the output
};

//
// PID Controller Class Definition
//
class PIDLib_PID
{
public:
    PIDLib_PID();
    PIDLib_PID(PIDLib_PIDSetupType& setupIn);
    ~PIDLib_PID();
    
    void init(PIDLib_PIDSetupType& setupIn);
    float run(float& setpoint, PIDLib_MeasurementType& measIn);

private:
    bool  m_firstPass;
    bool  m_antiwindupStatus;
    float m_myTime_sec;
    float m_valueP;
    float m_valueI;
    float m_valueD;
    float m_gainP;
    float m_gainI;
    float m_gainD;
    float m_outputLowerLimit;
    float m_outputUpperLimit;
    float m_outputUnlimited;
    float m_outputLimited;
    float m_setpoint;
    float m_errorPrevious;
    float m_error;
    float m_errorIntegral;
    float m_errorDerivative;
    float m_timeDelta_sec;
    float m_measTime_sec;
    float m_measValue;
};

PIDLib_PID::PIDLib_PID()
{
    m_firstPass          = true;
    m_antiwindupStatus   = false;
    m_myTime_sec         = 0.0f;
    m_valueP             = 0.0f;
    m_valueI             = 0.0f;
    m_valueD             = 0.0f;
    m_gainP              = 0.0f;
    m_gainI              = 0.0f;
    m_gainD              = 0.0f;
    m_outputLowerLimit   = 0.0f;
    m_outputUpperLimit   = 0.0f;
    m_outputUnlimited    = 0.0f;
    m_outputLimited      = 0.0f;
    m_setpoint           = 0.0f;
    m_errorPrevious      = 0.0f;
    m_error              = 0.0f;
    m_errorIntegral      = 0.0f;
    m_errorDerivative    = 0.0f;
    m_timeDelta_sec      = 0.0f;
    m_measTime_sec       = 0.0f;
    m_measValue          = 0.0f;

    return;
}

PIDLib_PID::PIDLib_PID(PIDLib_PIDSetupType& setupIn)
{
    m_firstPass          = true;
    m_antiwindupStatus   = false;
    m_myTime_sec         = 0.0f;
    m_valueP             = 0.0f;
    m_valueI             = 0.0f;
    m_valueD             = 0.0f;
    m_outputUnlimited    = 0.0f;
    m_outputLimited      = 0.0f;
    m_setpoint           = 0.0f;
    m_errorPrevious      = 0.0f;
    m_error              = 0.0f;
    m_errorIntegral      = 0.0f;
    m_errorDerivative    = 0.0f;
    m_timeDelta_sec      = 0.0f;
    m_measTime_sec       = 0.0f;
    m_measValue          = 0.0f;

    m_gainP             = setupIn.gainP;
    m_gainI             = setupIn.gainI;
    m_gainD             = setupIn.gainD;
    m_outputLowerLimit  = setupIn.outputLowerLimit;
    m_outputUpperLimit  = setupIn.outputUpperLimit;

    return;
}

PIDLib_PID::~PIDLib_PID()
{
    return;
}

void PIDLib_PID::init(PIDLib_PIDSetupType& setupIn)
{
    m_gainP             = setupIn.gainP;
    m_gainI             = setupIn.gainI;
    m_gainD             = setupIn.gainD;
    m_outputLowerLimit  = setupIn.outputLowerLimit;
    m_outputUpperLimit  = setupIn.outputUpperLimit;

    return;
}

float PIDLib_PID::run(float& setpoint, PIDLib_MeasurementType& measIn)
{
    // Read setpoint command
    m_setpoint = setpoint;

    // Read measurement information
    m_measTime_sec = measIn.time_sec;
    m_measValue    = measIn.value;

    // Compute time delta (from last measurement)
    m_timeDelta_sec = m_measTime_sec - m_myTime_sec;

    // Compute setpoint error
    m_error = m_setpoint - m_measValue;

    // Compute error integral
    if (m_firstPass)
        m_errorIntegral = 0.0f;
    else
        m_errorIntegral += (m_gainI == 0.0f || m_antiwindupStatus == false) ? 0.0f : m_timeDelta_sec * m_error;

    // Compute error derivative
    if (m_firstPass)
        m_errorDerivative = 0.0f;
    else
        m_errorDerivative = (m_gainD == 0.0f) ? 0.0f : (m_error - m_errorPrevious) / m_timeDelta_sec;

    // Compute values of proportional, integral, and derivative partitions
    m_valueP = m_gainP * m_error;
    m_valueI = (m_gainI == 0.0f) ? 0.0f : m_gainI * m_errorIntegral;
    m_valueD = (m_gainD == 0.0f) ? 0.0f : m_gainD * m_errorDerivative;

    // Unlimited output value
    m_outputUnlimited = m_valueP + m_valueI + m_valueD;

    // Limit output value
    if (m_outputUnlimited < m_outputLowerLimit)
    {
        m_outputLimited = m_outputLowerLimit;
        m_antiwindupStatus = (m_gainI == 0.0f) ? false : true;
    }
    else if (m_outputUnlimited > m_outputUpperLimit)
    {
        m_outputLimited = m_outputUpperLimit;
        m_antiwindupStatus = (m_gainI == 0.0f) ? false : true;
    }
    else
    {
        m_outputLimited = m_outputUnlimited;
        m_antiwindupStatus = false;
    }

    // Close out run routine
    m_myTime_sec = m_measTime_sec;
    m_errorPrevious = m_error;
    m_firstPass = false;

    return m_outputLimited;
}

#endif