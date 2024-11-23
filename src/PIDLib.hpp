#ifndef PID_LIB_HPP
#define PID_LIB_HPP

namespace PIDLib
{
    namespace CONST
    {
        double USEC_2_SEC = 1.0e-6;
    }

    //
    // Measurement Type Struct
    //
    struct MeasurementType
    {
        MeasurementType() :
            time_usec(0),
            value(0.0)
        {};

        unsigned long time_usec;  // time of measurement in micro seconds
        double        value;      // value of the measurement
    };

    //
    // PID Controller Setup Struct
    //
    struct SetupType
    {
        SetupType() :
            gainP(0.0),
            gainI(0.0),
            gainD(0.0),
            outputLowerLimit(0.0),
            outputUpperLimit(0.0)
        {};

        double gainP;             // proportional gain
        double gainI;             // integral     gain
        double gainD;             // derivative   gain
        double outputLowerLimit;  // desired lower limit of the output
        double outputUpperLimit;  // desired upper limit of the output
    };

    //
    // PID Controller Class Definition
    //
    class PID
    {
    public:
        PID();
        ~PID();

        void   reset();
        void   begin(SetupType& setupIn);
        void   config(SetupType& setupIn);
        void   setpoint(double& setpoint);
        double run(MeasurementType& measIn);

    private:
        void storeMeasInfo(MeasurementType& measIn);
        void computeCalculusTerms();
        void computeOutput();
        void exitRunRoutine();

        bool          m_firstPass;
        bool          m_antiwindupStatus;
        double        m_valueP;
        double        m_valueI;
        double        m_valueD;
        double        m_gainP;
        double        m_gainI;
        double        m_gainD;
        double        m_outputLowerLimit;
        double        m_outputUpperLimit;
        double        m_outputUnlimited;
        double        m_outputLimited;
        double        m_setpoint;
        double        m_errorPrevious;
        double        m_error;
        double        m_errorIntegral;
        double        m_errorDerivative;
        double        m_timeDelta_sec;
        double        m_measValuePrev;
        double        m_measValue;
        unsigned long m_measTime_usec;
        unsigned long m_myTime_usec;
    };

    PID::PID()
    {
        reset();
        return;
    }

    PID::~PID()
    {
        return;
    }

    void PID::reset()
    {
        m_firstPass        = true;
        m_antiwindupStatus = false;
        m_valueP           = 0.0;
        m_valueI           = 0.0;
        m_valueD           = 0.0;
        m_gainP            = 0.0;
        m_gainI            = 0.0;
        m_gainD            = 0.0;
        m_outputLowerLimit = 0.0;
        m_outputUpperLimit = 0.0;
        m_outputUnlimited  = 0.0;
        m_outputLimited    = 0.0;
        m_setpoint         = 0.0;
        m_errorPrevious    = 0.0;
        m_error            = 0.0;
        m_errorIntegral    = 0.0;
        m_errorDerivative  = 0.0;
        m_timeDelta_sec    = 0.0;
        m_measValuePrev    = 0.0;
        m_measValue        = 0.0;
        m_measTime_usec    = 0;
        m_myTime_usec      = 0;

        return;
    }

    void PID::begin(SetupType& setupIn)
    {
        // Reset all variables to zero
        reset();

        // Configure PID
        config(setupIn);

        return;
    }

    void PID::config(SetupType& setupIn)
    {
        m_gainP = setupIn.gainP;
        m_gainI = setupIn.gainI;
        m_gainD = setupIn.gainD;
        m_outputLowerLimit = setupIn.outputLowerLimit;
        m_outputUpperLimit = setupIn.outputUpperLimit;

        return;
    }

    void PID::setpoint(double& setpoint)
    {
        m_setpoint = setpoint;

        return;
    }

    double PID::run(MeasurementType& measIn)
    {
        // Read measurement info
        storeMeasInfo(measIn);

        // Time delta, derivative, and integral
        computeCalculusTerms();

        // Calculate output, impose limits, decide antiwindup trigger event
        computeOutput();

        // Some overhead before exiting
        exitRunRoutine();

        return m_outputLimited;
    }

    void PID::storeMeasInfo(MeasurementType& measIn)
    {
        // Read measurement information
        m_measTime_usec = measIn.time_usec;
        m_measValue = measIn.value;

        return;
    }

    void PID::computeCalculusTerms()
    {
        // Compute time delta (from last measurement)
        m_timeDelta_sec = CONST::USEC_2_SEC * ((double)(m_measTime_usec - m_myTime_usec));

        // Compute setpoint error
        m_error = m_setpoint - m_measValue;

        // Compute integral and derivative
        if (!m_firstPass)
        {
            m_errorIntegral += (m_gainI == 0.0 || m_antiwindupStatus == false) ? 0.0 : m_timeDelta_sec * m_error;
            m_errorDerivative = (m_gainD == 0.0) ? 0.0 : (m_measValue - m_measValuePrev) / m_timeDelta_sec;
        }
        else
        {
            m_firstPass = false;
            m_errorIntegral = 0.0;
            m_errorDerivative = 0.0;
        }

        return;
    }

    void PID::computeOutput()
    {
        // Compute values of proportional, integral, and derivative partitions
        m_valueP = m_gainP * m_error;
        m_valueI = (m_gainI == 0.0) ? 0.0 : m_gainI * m_errorIntegral;
        m_valueD = (m_gainD == 0.0) ? 0.0 : m_gainD * m_errorDerivative;

        // Unlimited output value
        m_outputUnlimited = m_valueP + m_valueI + m_valueD;

        // Limit output value
        if (m_outputUnlimited < m_outputLowerLimit)
        {
            m_outputLimited = m_outputLowerLimit;
            m_antiwindupStatus = (m_gainI == 0.0) ? false : true;
        }
        else if (m_outputUnlimited > m_outputUpperLimit)
        {
            m_outputLimited = m_outputUpperLimit;
            m_antiwindupStatus = (m_gainI == 0.0) ? false : true;
        }
        else
        {
            m_outputLimited = m_outputUnlimited;
            m_antiwindupStatus = false;
        }

        return;
    }

    void PID::exitRunRoutine()
    {
        // Close out run routine
        m_myTime_usec = m_measTime_usec;
        m_errorPrevious = m_error;
        m_measValuePrev = m_measValue;

        return;
    }
}

#endif