This is the documentation for my OSC to CV receiver eurorack module

It is meant to receive data sent from Pipo Interfaces modules
https://www.crowdsupply.com/pipo-interfaces/pipo
https://pipointerfaces.com/

The current HW uses an Esp32-s2 wemos mini module and has 4 CV outputs 0-10V
The hardware is still in a prototype stage.

Operation:
When turned on, the module creates a Wifi access point
The sender should connect to it and send data to it on port 8000
OSC data adresses of the 4 channels are "/cv/a","/cv/b","/cv/c","/cv/d"

![OSC-to-CV proto](/image.jpeg)



