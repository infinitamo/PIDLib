/////////////////////////////////////////////////////////////////////////////////////
//
//  MIT License
//  
//  Copyright(c) 2024 infinitamo
//  
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this softwareand associated documentation files(the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions :
//  
//  The above copyright noticeand this permission notice shall be included in all
//  copies or substantial portions of the Software.
//  
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  SOFTWARE.
// 
//***********************************************************************************
//***********************************************************************************
//!
//! @file: PIDLib.hpp
//! 
//! @brief: PID Library header file
//! 
//! @author: Austin M. Ottaway
//! 
//! @group: PIDLib
//! 
/////////////////////////////////////////////////////////////////////////////////////

#ifndef PID_LIB_HPP
#define PID_LIB_HPP

namespace PIDLib
{
    // Constants & Unit Conversions
    namespace CONST
    {
        constexpr double USEC_2_SEC = 1.0e-6;
    }

    // Error Mode Enumeration
    enum ONX_MODE
    {
        ERROR = 0,
        MEASR = 1
    };

    //============================================================
    //! @struct: MeasurementType
    //! 
    //! @brief: Defines struct for measurement containing its
    //!         value and timestamp
    //============================================================
    struct MeasurementType
    {
        MeasurementType() :
            time_usec(0),
            value(0.0)
        {};

        unsigned long time_usec;  // time of measurement in micro seconds
        double        value;      // value of the measurement
    };

    //============================================================
    //! @struct: SetupType
    //! 
    //! @brief: Defines struct for setting up PID options, gains,
    //!         and output limits 
    //============================================================
    struct SetupType
    {
        SetupType() :
            PonX(ONX_MODE::ERROR),
            IonX(ONX_MODE::ERROR),
            DonX(ONX_MODE::MEASR),
            gainP(0.0),
            gainI(0.0),
            gainD(0.0),
            outputLowerLimit(0.0),
            outputUpperLimit(0.0)
        {};

        char   PonX;              // mode - proportional on error or measurement
        char   IonX;              // mode - integral     on error or measurement
        char   DonX;              // mode - derivative   on error or measurement
        double gainP;             // proportional gain
        double gainI;             // integral     gain
        double gainD;             // derivative   gain
        double outputLowerLimit;  // desired lower limit of the output
        double outputUpperLimit;  // desired upper limit of the output
    };

    //============================================================
    //! @class: PID
    //! 
    //! @brief: PID Controller Class Definition
    //============================================================
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
        void computePIDTerms();
        void computeOutput();
        void exitRunRoutine();

        char          m_PonX;              // Flag - proportional on error or measurement
        char          m_IonX;              // Flag - integral     on error or measurement
        char          m_DonX;              // Flag - derivative   on error or measurement
        bool          m_firstRun;          // Mode - tracks first run occurence
        bool          m_clampingActive;    // Mode - integral clamping status
        bool          m_proActive;         // Mode - proportional term activity
        bool          m_intActive;         // Mode - integral     term activity
        bool          m_derActive;         // Mode - derivative   term activity
        double        m_gainP;             // Gain of the proportional portion of output
        double        m_gainI;             // Gain of the integral     portion of output
        double        m_gainD;             // Gain of the derivative   portion of output
        double        m_outputLowerLimit;  // Lower limit imposed on output
        double        m_outputUpperLimit;  // Upper limit imposed on output
        double        m_outputUnlim;       // Unlimited output
        double        m_outputLimited;     // Limited output (what is actually outputted)
        double        m_setpoint;          // Setpoint command
        double        m_errorPrevious;     // Previous value of the error (setpoint - measurement)
        double        m_error;             // Error term (setpoint - measurement)
        double        m_proTerm;           // Proportional term (no gain applied yet)
        double        m_intTerm;           // Integral     term (no gain applied yet)
        double        m_derTerm;           // Derivative   term (no gain applied yet)
        double        m_proTermGained;     // Proportional term, gains applied
        double        m_intTermGained;     // Integral     term, gains applied
        double        m_derTermGained;     // Derivative   term, gains applied
        double        m_timeDelta_sec;     // Delta between present and previous times, seconds
        double        m_measValuePrev;     // Previous measurement value
        double        m_measValue;         // Measurement value
        unsigned long m_measTime_usec;     // Measurement timestamp, micro-seconds
        unsigned long m_myTime_usec;       // PID time-of-validity, micro-seconds
    };
}

#endif