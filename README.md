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
