@rem ..\..\lazarus64\lazbuild.exe burnProject.lpi --bm=default
@rem ..\..\lazarus64\fpc\3.2.2\bin\x86_64-win64\delp.exe -r .
..\..\lazarus64\lazbuild.exe burnProject.lpi --bm=mt
..\..\lazarus64\fpc\3.2.2\bin\x86_64-win64\delp.exe -r .
@if not "%1"=="upx" goto end
upx.exe --best burn.exe 
:end