
from EasyLD2420 import LD2420

radar = LD2420(uart_pins=[21,20], ot_pin=0)                
radar.start()     

presence, grid, distance, extra = radar.wait_for_data()
print(f'received new data: {presence} \n{distance} \n\n{grid}')
