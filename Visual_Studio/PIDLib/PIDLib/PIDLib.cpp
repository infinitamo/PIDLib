/////////////////////////////////////////////////////////////////////////////////////
//
//  MIT License
//  
//  Copyright(c) 2024 infinitamo
//  
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files(the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions :
//  
//  The above copyright notice and this permission notice shall be included in all
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
//! @file: PIDLib.cpp
//! 
//! @brief: PID Library source file
//! 
//! @details: This PIDLib file contains source code that defines a PID Controller
//!           class that has extensive capabilities such as PonX, DonX, and IonX
//!           mode settings, antiwindup-clamping logic to protect against integral
//!           windup during moments of output saturation, and ... more to come!
//! 
//! @author: Austin M. Ottaway
//! 
//! @group: PIDLib
//! 
/////////////////////////////////////////////////////////////////////////////////////

#include <cmath>       // C++ standard math library
#include "PIDLib.hpp"  // PIDLib header file include

//============================================================
//! @function: PID
//! 
//! @brief: PID Class Constructor
//! 
//! @return: n/a
//============================================================
PIDLib::PID::PID()
{
    reset();
    return;
}

//============================================================
//! @function: ~PID
//! 
//! @brief: PID Class Destructor
//!
//! @return: n/a
//============================================================
PIDLib::PID::~PID()
{
    return;
}

//============================================================
//! @function: reset
//! 
//! @brief: Resets all member variables
//! 
//! @return: void
//============================================================
void PIDLib::PID::reset()
{
    m_PonX             = ONX_MODE::ERROR;  // default to error
    m_IonX             = ONX_MODE::ERROR;  // default to error
    m_DonX             = ONX_MODE::MEASR;  // default to measurement
    m_firstRun         = true;
    m_clampingActive   = false;
    m_proActive        = false;
    m_intActive        = false;
    m_derActive        = false;
    m_gainP            = 0.0;
    m_gainI            = 0.0;
    m_gainD            = 0.0;
    m_outputLowerLimit = 0.0;
    m_outputUpperLimit = 0.0;
    m_outputUnlim      = 0.0;
    m_outputLimited    = 0.0;
    m_setpoint         = 0.0;
    m_errorPrevious    = 0.0;
    m_error            = 0.0;
    m_proTerm          = 0.0;
    m_intTerm          = 0.0;
    m_derTerm          = 0.0;
    m_proTermGained    = 0.0;
    m_intTermGained    = 0.0;
    m_derTermGained    = 0.0;
    m_timeDelta_sec    = 0.0;
    m_measValuePrev    = 0.0;
    m_measValue        = 0.0;
    m_measTime_usec    = 0;
    m_myTime_usec      = 0;

    return;
}

//============================================================
//! @function: begin
//! 
//! @brief: Resets all variables and sets config
//! 
//! @return: void
//============================================================
void PIDLib::PID::begin(SetupType& setupIn)
{
    // Reset all variables to zero
    reset();

    // Configure PID
    config(setupIn);

    return;
}

//============================================================
//! @function: config
//! 
//! @brief: Configures the PID settings
//! 
//! @return: void
//============================================================
void PIDLib::PID::config(SetupType& setupIn)
{
    // Set ONX modes
    m_PonX = setupIn.PonX;
    m_IonX = setupIn.IonX;
    m_DonX = setupIn.DonX;

    // Set gains
    m_gainP = setupIn.gainP;
    m_gainI = setupIn.gainI;
    m_gainD = setupIn.gainD;

    // Set output limits
    m_outputLowerLimit = setupIn.outputLowerLimit;
    m_outputUpperLimit = setupIn.outputUpperLimit;

    // Set other internal modes
    if (m_gainP != 0.0) m_proActive = true;
    if (m_gainI != 0.0) m_intActive = true;
    if (m_gainD != 0.0) m_derActive = true;

    return;
}

//============================================================
//! @function: setpoint
//! 
//! @brief: Sets the PID setpoint command
//! 
//! @return: void
//============================================================
void PIDLib::PID::setpoint(double& setpoint)
{
    m_setpoint = setpoint;

    return;
}

//============================================================
//! @function: run
//! 
//! @brief: Primary driver routine for the PID class
//! 
//! @return: double
//============================================================
double PIDLib::PID::run(MeasurementType& measIn)
{
    // Read measurement info
    storeMeasInfo(measIn);

    // Time delta, proportional, derivative, and integral terms (ungained)
    computePIDTerms();

    // Calculate output, impose limits, decide antiwindup trigger event
    computeOutput();

    // Some overhead before exiting
    exitRunRoutine();

    return m_outputLimited;
}

//============================================================
//! @function: storeMeasInfo
//! 
//! @brief: This records the measurement timestamp and value
//! 
//! @return: void
//============================================================
void PIDLib::PID::storeMeasInfo(MeasurementType& measIn)
{
    // Read measurement information
    m_measTime_usec = measIn.time_usec;
    m_measValue = measIn.value;

    return;
}

//============================================================
//! @function: computePIDTerms
//! 
//! @brief: This computes the ungained PID terms depending on
//!         proportional, integral, and derivative activity
//! 
//! @return: void
//============================================================
void PIDLib::PID::computePIDTerms()
{
    // Compute time delta (from last measurement)
    m_timeDelta_sec = CONST::USEC_2_SEC * ((double)(m_measTime_usec - m_myTime_usec));

    // Compute setpoint error
    m_error = m_setpoint - m_measValue;

    // Compute proportional, integral, and derivative terms (gains not applied yet)
    if (!m_firstRun)
    {
        if (m_proActive)
        {
            if (m_PonX == ONX_MODE::ERROR)
                m_proTerm = m_error;       // PonE
            else if (m_PonX == ONX_MODE::MEASR)
                m_proTerm = -m_measValue;  // PonM
            else
                m_proTerm = m_error;       // PonE default
        }

        if (m_intActive)
        {
            if (m_IonX == ONX_MODE::ERROR)
                m_intTerm += m_error * m_timeDelta_sec;       // IonE
            else if (m_IonX == ONX_MODE::MEASR)
                m_intTerm += -m_measValue * m_timeDelta_sec;  // IonM
            else
                m_intTerm += m_error * m_timeDelta_sec;       // IonE default
        }

        if (m_derActive)
        {
            double tol = 1e-12;  // more than enough to account for micro-second precision
            double dt = m_timeDelta_sec;
            if (fabs(dt) < tol) dt = tol;  // clamp time delta to prevent NaN

            if (m_DonX == ONX_MODE::MEASR)
                m_derTerm = -(m_measValue - m_measValuePrev);  // DonM
            else if (m_DonX == ONX_MODE::ERROR)
                m_derTerm = (m_error - m_errorPrevious);       // DonE
            else
                m_derTerm = -(m_measValue - m_measValuePrev);  // DonM default

            m_derTerm /= dt;
        }
    }
    else
    {
        m_firstRun = false;
    }

    return;
}

//============================================================
//! @function: computeOutput
//! 
//! @brief: Computes the output of the PID controller and
//!         detects the necessary conditions for clamping of
//!         the integral value to become active and prevent
//!         windup
//! 
//! @return: void
//============================================================
void PIDLib::PID::computeOutput()
{
    // Compute gained values of proportional, integral, and derivative terms
    if (m_proActive) m_proTermGained = m_gainP * m_proTerm;
    if (m_intActive) m_intTermGained = m_gainI * m_intTerm;
    if (m_derActive) m_derTermGained = m_gainD * m_derTerm;

    // Unlimited output value
    m_outputUnlim = 0.0;
    if (m_proActive) m_outputUnlim += m_proTermGained;
    if (m_intActive) m_outputUnlim += m_intTermGained;
    if (m_derActive) m_outputUnlim += m_derTermGained;

    // Limit output value
    if (m_outputUnlim < m_outputLowerLimit)
    {
        m_outputLimited = m_outputLowerLimit;

        if (m_intActive) m_clampingActive = true;  // for integral antiwindup, output saturated - turn ON clamping
    }
    else if (m_outputUnlim > m_outputUpperLimit)
    {
        m_outputLimited = m_outputUpperLimit;

        if (m_intActive) m_clampingActive = true;  // for integral antiwindup, output saturated - turn ON clamping
    }
    else
    {
        m_outputLimited = m_outputUnlim;

        if (m_intActive) m_clampingActive = false;  // for integral antiwindup, output within limits - turn OFF clamping
    }

    return;
}

//============================================================
//! @function: exitRunRoutine
//! 
//! @brief: Overhead logic at the end of run
//! 
//! @return: void
//============================================================
void PIDLib::PID::exitRunRoutine()
{
    // Close out run routine
    m_myTime_usec = m_measTime_usec;
    m_errorPrevious = m_error;
    m_measValuePrev = m_measValue;

    return;
}