::del factory_firmware.hex/s 

merge\mergehex.exe --merge hex\s110_nrf51822_6.0.0-5.beta_softdevice.hex   hex\app.hex        --output hex\withoutbootloader.hex
merge\mergehex.exe --merge hex\withoutbootloader.hex                       hex\bootloader.hex --output hex\withoutsn.hex 
merge\mergehex.exe --merge hex\withoutsn.hex                               hex\sn.hex         --output factory_firmware.hex 

del hex\withoutbootloader.hex/s 
del hex\withoutsn.hex/s 

::pause
