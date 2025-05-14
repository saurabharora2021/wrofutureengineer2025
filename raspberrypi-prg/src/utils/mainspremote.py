import spremote

hub = spremote.Hub('/dev/ttyACM0')

print('light up one pixel')
lm = spremote.LightMatrix(hub)
lm.set_pixel(2, 2, 100)

print('change power button color')
power_button = spremote.Button(hub, 'POWER')
power_button.set_color(9)

hub.disconnect()