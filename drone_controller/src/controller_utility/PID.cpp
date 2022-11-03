/*
 * Copyright 2020 Kangyao Huang, ACSE, The University of Sheffield, UK
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define M_PI       3.14159265358979323846
#define wrap_180(x) (x < -M_PI ? x+(2*M_PI) : (x > M_PI ? x - (2*M_PI): x))
#include "PID.h"


namespace drone_control{

PID::PID(){}
PID::~PID(){}

void PID::SetGainParameters(double params[5])
{
    G = params[0];
    Kp = params[1];
    Ki = params[2];
    Kd = params[3];
    Ki_max = params[4];
}

void PID::ReSet()
{
    errorPrev = 0;
    integrator = 0;
}

double PID::ComputeCorrection(double cmd, double pos, double loopTime)
{
    double correction = 0;
    error = cmd - pos;

    if (Ki_max != 0)
    {
         if (integrator >= Ki_max)  
        {
            integrator = Ki_max;
        }else if (integrator <= - Ki_max)
        {
            integrator = - Ki_max;
        }else
        {
            integrator += error;
        }
        
    }else
    {
        integrator += error;
    }
    
    //integrator += error;
    correction = G * (Kp * error + Ki * integrator + Kd * (error - errorPrev)/(loopTime));

    errorPrev = error;
    return correction;
}

double PID::ComputeCorrectionLimit(double cmd, double pos, double loopTime)
{
    double correction = 0;
    error = cmd -pos;
    error = wrap_180(error);
    if (Ki_max != 0)
    {
         if (integrator >= Ki_max)  
        {
            integrator = Ki_max;
        }else if (integrator <= - Ki_max)
        {
            integrator = - Ki_max;
        }else
        {
            integrator += error;
        }
        
    }else
    {
        integrator += error;
    }
    //integrator += error;
    correction = G * (Kp * error + Ki * integrator + Kd * (error - errorPrev)/(loopTime));

    errorPrev = error;
    return correction;
}



}