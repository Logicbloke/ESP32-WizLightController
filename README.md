# Project Overview

This project utilizes the following components to build a light control system with multiple modes of operation:

## Components
- **Red Button**: Connected to GND and GPIO4  
- **Blue Button**: Connected to GND and GPIO5  
- **LCD**: Connected to GND, 5V, GPIO17 (SCL), GPIO18 (SDA)  
- **Potentiometer**: Middle pin connected to ADC1CH5/PIN6, others to 3V3 and GND  
- **Photoresistor (LDR)**:  
  - Connected as: 5V --- LDR --- ADC2CH2/PIN13  
  - Voltage Divider: LDR connected to a 10KΩ resistor that bridges to GND  
- **PIR/Motion Sensor**:  
  - Middle pin connected to GPIO19, others to GND and 5V  

---

## Functionality

### Buttons
- **Red Button**: Cycles through power modes:  
  - **OFF**: Turns off the light but allows adjustments to brightness or light mode.  
  - **ON**: Brightness is controlled using the potentiometer (values mapped from 0-4095 to 0-100%).  
  - **AUTO**: Lights turn on as long as motion is detected (signal HIGH from PIR sensor).
    - The duration of the HIGH signal can be adjusted on the PIR sensor (up to 250 seconds).

- **Blue Button**: Switches between light modes:  
  - COOL WHITE  
  - DAYLIGHT  
  - NIGHT LIGHT  
  - WARM WHITE  

### Automatic Brightness Adjustment
- In **AUTO mode**, brightness is determined by the LDR instead of the potentiometer.  
- The LDR value reflects ambient light:  
  - Values are normalized from 0-4095 to a range of 0-100.  
  - Brightness is calculated as `100 - normalized_value`.

#### Example:
- If the LDR detects maximum light (value = 4095):  
  - Normalized value = 100.  
  - Brightness = 0% (light is dimmed).  
- If the LDR detects no light (value = 0):  
  - Normalized value = 0.  
  - Brightness = 100% (light is fully bright).
