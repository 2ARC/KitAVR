avrdude -c dasa -p atmega162 -P com6 -U lfuse:w:0xef:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m
pause