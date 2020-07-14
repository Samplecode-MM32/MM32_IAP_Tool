@echo off


set /p ttt=请输入版本:

::echo %date:~0,4%%date:~5,2%%date:~8,2%_%time:~0,2%%time:~3,2%%time~6,2%V%ttt%>> name.txt

echo %date:~0,4%%date:~5,2%%date:~8,2%_%ttt%>> name.txt

::得到系统日期和时间

for /f "delims=" %%a in (name.txt) do (
merge\mergehex.exe --merge .\hex\bootloader.hex                               .\hex\app.hex         --output %%a.hex 
)

.\hex\hex2bin.exe  .\hex\app.hex hex\app.bin

::ren "hex\app.bin" "%date:~0,4%%date:~5,2%%date:~8,2%_%time:~0,2%%time:~3,2%%time~6,2%V%ttt%.bin" 
ren "hex\app.bin" %date:~0,4%%date:~5,2%%date:~8,2%_%ttt%.bin" 

move .\hex\*.bin .\

del name.txt/s 


#pause
