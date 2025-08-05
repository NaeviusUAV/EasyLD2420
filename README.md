# EasyLD2420
Easy to use micropython library / driver to use the HLK-LD2420 24Ghz Radar sensor (human presence radar) with basic python skills.

## usage example

```python
from EasyLD2420 import LD2420

radar = LD2420(uart_pins=[21,20], ot_pin=0)                
radar.start()     

presence, grid, distance, extra = radar.wait_for_data()
print(f'LD2420 data received! {presence} \n{distance} \n\n{grid}')
```

## docs

### Quick start
1. Upload this library onto your micropython device:
    - make a new file, paste code from EasyLD2420.py
    - Save it (ctrl-s) as ServoRX.py
    - make a new file i.e. main.py to use it in   
2. Setting up the radar
   - import LD2420 from the file you just saved:
     ```python
     from EasyLD2420 import LD2420
     ```
   - make an instance with the defined pins
     ```python
      radar = LD2420(uart_pins=[21,20], ot_pin=0)                 
     ```
   - start the radar (puts it in the mode where it returns data to the microcontroller
     ```python
      radar.start()
       ```
    - then, print out all available functions like so:
      ```python
      radar.help()
      ```
3. reading the radar
   - there are 4 variables returned when using wait_for_data()
     ```python
     presence, grid, distance, extra = radar.wait_for_data()
     ```
     when printed it looks like this:
    ``presence:  True ``
    ``4x4 radar grid (energy values found): [[58237, 59261, 74, 73], [29, 45, 13, 45], [20, 13, 16, 13], [18, 20, 26, 13]] ``
    ``distance:  16`` (in cm)
    ``extra data returned: {'raw': b''}``

### All functions

#### basics
  - `start()`
    
      sends a command to LD2420 to start sending back data
  - `stop()`
    
      sends a command to LD2420 to stop sending back data
  - `help()`
    
      prints out this list of functions available
  - `reset()`
    
      restarts the LD2420 board.
      PS: it will do so already automatically if it receives a bad stream of data for too long on wait_for_data() function call.
  - `wait_for_data()`
    
      catches a valid bunch of bytes from the LD2420, decodes that into python variables and returns it.
      its not concurrent so on call it will not return and your code wont either until theres data (which is very quick, as LD2420 is capable of 10hz)
      it returns presence bool, a list of lists showing the grid of energy it found, an int for distance, and dict for additional data it wants to tell you about lol.
  - `get_version()`
    
      asks the LD2420 to tell us what firmware version is running. with the soldering pads you can appearently change that.
      it will return the firmware version in a string like '2.0'
    
  #### Firmware customisation
  - `set_sensitivity(level)`
    
      change the sensitivity of the radar between 0 and 5.
      will send the command in bytes to the LD2420
  - `set_refresh_rate(rate_hz, rate_ms)`

      change the rate at which the LD2420 should send back the data.
      choose to use hz or ms to define this. It will use the one you passed in
      and writes the bytes to the LD2420 regs
  - `set_detection_range(min_cm, max_cm)`

      The radar module works with 16 bins (FFT sorts them). each bin is roughly 70 cm worth of range.
      This function will tell the DSP which bins can trigger presence.
      Gates outside the window are still reported in the 4x4 grid, but they don’t influence the presence or the distance returned.
      set min and max values in centimeters

#### Advanced
  - `write_register(reg_addr: int, value: int, verify: bool)`
  
    write your own commands to the LD2420, must be 32 bit val
    it will send the command, then read the reg if verify=true to check if its changed properly.
  - `read_register(reg_addr: int)`

    read the passed register and returns the value

A guide on settings, and tuning. would love if people incorporate more of the possibilities in this repo.
https://github.com/GelidusResearch/device.docs/blob/main/ps1.guides/PS1-Tuning-Guide-LD2420.md

Todo:
- implement all settings available as seen on the guide
