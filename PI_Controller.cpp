/*
      PI Controller
      Copyright (C) 2020  Phillip J Schmidt

         This program is free software: you can redistribute it and/or modify
         it under the terms of the GNU General Public License as published by
         the Free Software Foundation, either version 3 of the License, or
         (at your option) any later version.

         This program is distributed in the hope that it will be useful,
         but WITHOUT ANY WARRANTY; without even the implied warranty of
         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
         GNU General Public License for more details.

         You should have received a copy of the GNU General Public License
         along with this program.  If not, see <http://www.gnu.org/licenses/>

 */


#include "PI_Controller.h"


PI_Controller::PI_Controller( float K_a, float K_b, int loopFreq, float minSat, float maxSat )
{
   output = 0;
   integrator = 0;
   loopPeriod = 1.0f / float(loopFreq);

   setKa( K_a );
   setKb( K_b );
   setMinSaturation( minSat );
   setMaxSaturation( maxSat );
}


void PI_Controller::setKa(float K_a)
{
   Ka = K_a;
}

void PI_Controller::setKb(float Kb)
{
   Kb_dt = Kb * loopPeriod;  // adjust integration constant for loop frequency
}

void PI_Controller::setMinSaturation(float minSat)
{
   minSaturation = minSat;
}

void PI_Controller::setMaxSaturation(float maxSat)
{
   maxSaturation = maxSat;
}

float PI_Controller::update( const float & processVariable, const float & setPoint )
{
   // Serial PI controller
   float error = setPoint - processVariable;
   float prop = constrain( Ka * error, minSaturation, maxSaturation ); // prevent over saturation of proportional term
   integrator += Kb_dt * prop;
   integrator = constrain( integrator, minSaturation - prop, maxSaturation - prop ); // dynamically clamp integrator as proportional appoaches saturation
   output = prop + integrator;
   return output;
}

float PI_Controller::getOutput()
{
   return output;
}
