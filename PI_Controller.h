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

      private:
         float Ka;
         float Kb_dt, integrator;
         float minSaturation, maxSaturation;
         float output; 
         float loopPeriod;
   };

#endif