# Serial PI Controller with dynamic integrator clamping

![Image of Serial PI Controller topology](https://e2e.ti.com/cfs-file.ashx/__key/communityserver-blogs-components-weblogfiles/00-00-00-07-88/4760.1.3.gif)

*image credit [TI.com](www.ti.com)*

For an example of the advantage of a serial PI controller over a traditional parallel controller, [read this](https://e2e.ti.com/blogs_/b/industrial_strength/archive/2015/07/20/teaching-your-pi-controller-to-behave-part-ii)

## Object Creation
```
PI_Controller my_PI_control( proportionalGain, IntegralGain, controlLoopFrequency, minControlLimit, maxControlLimit );
```

**proportionalGain** - This gain constant is for the proportional portion of the controller

**IntegralGain** - This gain regulates the integral response of the controller.

**controlLoopFrequency** - The is the frequency in Hz of that the PI controller is updated

**minControlLimit** - This is the minimum saturation limit of the control output.  The ouput will not be lower than this value.

**maxControlLimit** - This is the maximum saturation limit of the control output  The output will not be higher than this value.


## Controller Usage
```C
void myControlLoop() // run this at the controlLoopFrequency set at object creation
{
    processValue = getCurrentProcessValue();    // get the current process value from the system you are controlling
    setPoint = getCurrentSetPoint();            // get the current desired set point for the system
    float output = update( processValue, setPoint ); // pass processValue and setPoint into the PI controller, the updated result is returned as a float
    controlMySystem( output );                  // pass the output to your control scheme
}
```

### Behaviors:
* Output is always constrained to min and max saturation limits.
* Propotional response is given control priority during dynamic adjustments.
* Integral effect is dynamically clamped such that it will never be larger than the difference between the proportional response and the saturation limits.  This prevents integral wind-up and softly re-enables integral response as the controller recovers from strong disturbances and again nears steady state.

## Function List

`PI_Controller( float Ka, float Kb, float loopFreq, float minSat, float maxSat );` -- Object creation

`void setKa( float Ka );` -- update proportional gain constant

`void setKb( float Kb );` -- update integral gain constant

`void setMinSaturation( float minSat );` -- set the minimum saturation limit

`void setMaxSaturation( float maxSat );` -- set the maximum saturation limit

`float update( float processVariable, float setPoint );` -- PI controller update function

`float getOutput();` -- get the latest output without passing in new PV and SP


