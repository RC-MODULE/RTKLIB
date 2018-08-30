$scriptDir = Split-Path -Path $MyInvocation.MyCommand.Definition -Parent
$src = $scriptDir + "\..\bin"
$dst = $src + "\RTKLIB_bin.zip"
Remove-Item -Force -Recurse "$dst"
Compress-Archive "$src\*.exe" -CompressionLevel Optimal -DestinationPath "$dst"
