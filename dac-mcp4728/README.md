# Overview 
- The MCP4728 is a 12-bit quad output I2C DAC.
- This example implements the mcp4728 DAC using the inbuilt zephyr sensor drivers.
- A shell is implemented to allow for DAC writing from a serial terminal.
- The default voltage reference is the internal one (2.048V)

# Supported serial commands:
mcp4728 set <channel number 0-4> <value 0-4095>

e.g. mcp4728 set 0 1024 // this sets dac channel 0 to 1024 (one quater of full scale)

# Test & results
When I2C is configured with a speed of 400KHz, the fastest response time acheived to change a single DAC channel was 162us (verified using an oscilliscope). 
