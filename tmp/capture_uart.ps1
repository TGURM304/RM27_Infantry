param([string]$PortName, [string]$OutputPath, [int]$DurationSeconds)
$serial = [System.IO.Ports.SerialPort]::new($PortName, 2000000, 'None', 8, 'One')
$serial.ReadTimeout = 50
$serial.Open()
Start-Sleep -Milliseconds 500
$serial.DiscardInBuffer()
$file = [System.IO.File]::Open($OutputPath, [System.IO.FileMode]::Create, [System.IO.FileAccess]::Write, [System.IO.FileShare]::Read)
$chunk = [byte[]]::new(4096)
$timer = [Diagnostics.Stopwatch]::StartNew()
$total = 0
while ($timer.Elapsed.TotalSeconds -lt $DurationSeconds) {
    try {
        $count = $serial.Read($chunk, 0, $chunk.Length)
        $file.Write($chunk, 0, $count)
        $total += $count
    } catch [System.TimeoutException] { }
}
$file.Close()
$serial.Close()
Set-Content -LiteralPath ($OutputPath + '.size') -Value $total -Encoding ascii
