esptool.exe --chip esp32s3 --baud 921600  --before default_reset --after hard_reset write_flash  -z --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0 "bin\Monitoring_System_LCD.bootloader.bin" 0x8000 "bin\Monitoring_System_LCD.partitions.bin" 0xe000 "bin\boot_app0.bin" 0x10000 "bin\Monitoring_System_LCD.bin" 
pause