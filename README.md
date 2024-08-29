# I-Bike system
## Overview
- This system enhances the cycling experience by providing real-time data and control features, ensuring both safety and convenience for the rider.

## Features

### Speed and Distance Monitoring
- A magnet and a Hall sensor (3144E) are used to determine the RPM of the bike, allowing for the calculation of distance and speed.

### Direction Light Indicator
- An LED matrix module at the back of the bicycle displays a left or right indicator when the bike is changing directions.
- Direction changes are detected using a gyroscope placed on the steering wheel.

### Electric Auto-Control Derailleur 
- A servo motor controls the derailleur's movement by winding and unwinding the derailleur cable, allowing the user to make manual adjustments on the gear level based on the current slope.

### Real-time Derailleur Control
- The gyroscope's data is utilized to assess the riding situation (flat, downhill, uphill), automatically adjusting the derailleur to the appropriate level based on real-time conditions.

### Bluetooth Transmission
- An HC-05 module transmits speed and distance traveled to an Android phone.
- Users can manually turn the lights on or off by sending commands from the Android phone.

