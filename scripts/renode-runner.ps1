param (
    [Parameter(Mandatory=$true)]
    [string]$BinaryPath,
    [Parameter(Mandatory=$true)]
    [string]$RescPath
)

# Convert to absolute paths for Renode
$absBinary = (Resolve-Path $BinaryPath).Path
$absResc = (Resolve-Path $RescPath).Path

# Run Renode with the binary path passed as a variable (@ prefix for ReadFilePath)
renode --console -e "`$bin=@$absBinary; include @$absResc; start"
