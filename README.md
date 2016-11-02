# FLIR-Lepton-Calibration
A basic calibration for FLIR Lepton with MLX90614 sensor. 

Based on the document, the ambient temperature of the lepton itself will always be referred as the raw value 8192. The difference of the raw value and 8192 is kind of linear to the final temperature calculated. Once we know the actual temperature of the lepton itself and the slope of the function, we can calculate the temperature with a given raw value. 

In order to get the temperature of the lepton, I use the MLX90614 temperature sensor and assume that its ambient temperature is the same as the lepton. We will need a box to provide a stable environment for the sensors so that their temperatures will not vary a lot when the outside temperature changes. Therefore, we can use the equation below:

Temperature T = slope * (raw - 8192) + ambTemp

According to Max Ritter's work, the slope he calculated is 0.0217. I recorded the temperature of the difference between raw data and 8192 and the temperature measured by the MLX90614 to apply a polyfit using Matlab and I found the slope I got was 0.026. The result might vary when the environment temperature changes.

To simply calibrate, I also try the DS18B20 sensor since it is cheaper and smaller than the MLX90614. If well calibrated and put in a sealed space, the temperature of the sensor and the camera should be similar and therefore we don't need to know the temperature of an object via the temperature sensor.
