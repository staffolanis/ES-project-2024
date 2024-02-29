# Embedded System Project

## For the Board
To compile go to the principle directory and run
```angular2html
make -j$(nproc --all)
```


To upload the code on the board run
```angular2html
st-flash --reset write main.bin 0x8000000
```

## For the Serial
Using the board stm32f4discovery, connect to USART3 usingl **PB10 (TXD)** and **PB11 (RXD)**

To open the serial port run
```angular2html
screen /dev/ttyUSB0 19200
```