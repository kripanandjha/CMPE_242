The assignment is about validation adc value from potentio meter to a lpc1758 nod
after adding compensation function send this data from ARM server which will 
do FFT of input ADC data and generated power spectrum out of it and further validates
whether data has aliasing effect or not.

LPCNOD 1758
lpc1758 runs freertos from 244 class. You can run the node adc code in freertos
and flash in the board. LPCNOD gets the data from ADC source and add compensation
function and send it over uart to ARMSVR

ARMSVR 
server run in linux environment and given source is sample application which calls uart 
communication driver to send and receive the data from LPCNOD apply FFT and validates data.
