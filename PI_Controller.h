/*
      PI_Controller
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


#ifndef PI_Controller_h
   #define PI_Controller_h

   #ifndef constrain
      #define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))  // from Arduino.h
   #endif

   class PI_Controller
   {
      public:

         PI_Controller( float Ka, float Kb, int loopFreq, float minSat, float maxSat );

         void setKa( float Ka );
         void setKb( float Kb );
         void setMinSaturation( float minSat );
         void setMaxSaturation( float maxSat );

         float update( const float & processVariable, const float & setPoint );

         float getOutput();

      //private:
         float Ka;
         float Kb_dt, integrator;
         float minSaturation, maxSaturation;
         float output; 
         float loopPeriod;
   };

#endif