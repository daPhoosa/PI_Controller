/*
      PI_Controller

      MIT License

      Copyright (c) 2020 Phillip Schmidt

         Permission is hereby granted, free of charge, to any person obtaining a copy
         of this software and associated documentation files (the "Software"), to deal
         in the Software without restriction, including without limitation the rights
         to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
         copies of the Software, and to permit persons to whom the Software is
         furnished to do so, subject to the following conditions:

         The above copyright notice and this permission notice shall be included in all
         copies or substantial portions of the Software.

         THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
         IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
         FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
         AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
         LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
         OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
         SOFTWARE.

 */


#include "PI_Controller.h"


PI_Controller::PI_Controller( float K_a, float K_b, int loopFreq, float minSat, float maxSat )
{
   output = 0;
   i_term = 0;
   loopPeriod = 1.0f / float( loopFreq );

   setKa( K_a );
   setKb( K_b );
   setMinSaturation( minSat );
   setMaxSaturation( maxSat );
}


void PI_Controller::setKa( float K_a )
{
   Ka = K_a;
}

void PI_Controller::setKb( float Kb )
{
   Kb_dt = Kb * loopPeriod;  // adjust integration constant for loop frequency
}

void PI_Controller::setMinSaturation( float minSat )
{
   minSaturation = minSat;
}

void PI_Controller::setMaxSaturation( float maxSat )
{
   maxSaturation = maxSat;
}

float PI_Controller::update( const float & processVariable, const float & setPoint )
{
   float error = setPoint - processVariable;

   float p_term = Ka * error;

   if( p_term > maxSaturation )
   {
      i_term = 0;
      return maxSaturation;
   }
   
   if( p_term < minSaturation )
   {
      i_term = 0;
      return minSaturation;
   }

   i_term += Kb_dt * p_term;
   i_term = constrain( i_term, minSaturation - p_term, maxSaturation - p_term ); // dynamically clamp integrator as proportional appoaches saturation

   return p_term + i_term;
}
