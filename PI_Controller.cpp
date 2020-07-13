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


PI_Controller::PI_Controller( float Ka, float Kb, float loopFreq, float minSat, float maxSat )
{
   output = 0;
   integrator = 0;
   loopPeriod = 1.0f / loopFreq;

   setKa( Ka );
   setKb( Kb );
   setMinSaturation( minSat );
   setMaxSaturation( maxSat );
}


void PI_Controller::setKa(float Ka)
{
   K_a = Ka;
}

void PI_Controller::setKb(float Kb)
{
   K_b = Kb * loopPeriod;  // adjust integration constant for loop frequency
}

void PI_Controller::setMinSaturation(float minSat)
{
   minSaturation = minSat;
}

void PI_Controller::setMaxSaturation(float maxSat)
{
   maxSaturation = maxSat;
}

float PI_Controller::update( float processVariable, float setPoint )
{
   float error = setPoint - processVariable;
   float A = constrain( K_a * error, minSaturation, maxSaturation ); // prevent over saturation of proportional term
   integrator += K_b * A;
   integrator = constrain( integrator, minSaturation - A, maxSaturation - A ); // dynamically clamp integrator as proportional appoaches saturation
   output = A + integrator;
   //Serial.print(processVariable); Serial.print("\t");
   //Serial.print(setPoint);        Serial.print("\t");
   //Serial.print(A);               Serial.print("\t");
   //Serial.print(integrator); Serial.print("\t");
   //Serial.print(output); Serial.print("\t");
   //Serial.print(minSaturation); Serial.print("\t");
   //Serial.print(maxSaturation); Serial.print("\t");
   return output;
}

float PI_Controller::getOutput()
{
   return output;
}
