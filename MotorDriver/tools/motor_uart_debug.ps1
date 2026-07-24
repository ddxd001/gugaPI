# MotorDriver UART binary-frame debug helper.
# Protocol: 0xAA CMD REG LEN DATA... CRC8  (CRC8 poly=0x07 init=0x00 over SOF..DATA)
# CMD: 0x01 read, 0x02 write, 0x03 heartbeat. Response CMD = CMD|0x80.
# Read frame:     AA 01 REG <readlen> CRC              (LEN = bytes to read, no DATA)
# Write frame:    AA 02 REG <datalen> DATA... CRC       (LEN = data length)
# Heartbeat frame:AA 03 REG 00 CRC
# Usage:
#   . .\motor_uart_debug.ps1
#   Open-Motor COM4
#   Send-MotorPing
#   Read-MotorReg 0x00 6
#   Write-MotorReg 0x10 @(0x01,0x05,0x00)
#   Read-MotorStatus
#   Close-Motor

$script:MotorPort = $null

function Calc-Crc8 {
    param([byte[]]$Data)
    $crc = [byte]0
    foreach ($b in $Data) {
        $crc = $crc -bxor $b
        for ($i = 0; $i -lt 8; $i++) {
            if ($crc -band 0x80) { $crc = (($crc -shl 1) -bxor 0x07) -band 0xFF }
            else { $crc = ($crc -shl 1) -band 0xFF }
        }
    }
    return $crc
}

function Open-Motor {
    param([string]$Port = "COM4", [int]$Baud = 115200)
    if ($script:MotorPort -and $script:MotorPort.IsOpen) { $script:MotorPort.Close() }
    $script:MotorPort = New-Object System.IO.Ports.SerialPort $Port,$Baud,0,8,1
    $script:MotorPort.ReadTimeout = 800
    $script:MotorPort.WriteTimeout = 800
    $script:MotorPort.Open()
    Write-Host "[open] $Port @ $Baud 8N1"
}

function Close-Motor {
    if ($script:MotorPort -and $script:MotorPort.IsOpen) { $script:MotorPort.Close() }
    Write-Host "[close]"
}

function Send-RawFrame {
    param([byte[]]$Frame)
    if (-not ($script:MotorPort -and $script:MotorPort.IsOpen)) { throw "port not open" }
    $script:MotorPort.DiscardInBuffer()
    $buf = [byte[]]($Frame | ForEach-Object { [byte]$_ })
    $script:MotorPort.Write($buf, 0, $buf.Length)
}

function Read-MotorResponse {
    param([int]$TimeoutMs = 800)
    $bytes = New-Object System.Collections.Generic.List[byte]
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    while ($sw.ElapsedMilliseconds -lt $TimeoutMs) {
        while ($script:MotorPort.BytesToRead -gt 0) {
            $bytes.Add([byte]$script:MotorPort.ReadByte())
            if ($bytes.Count -ge 5) {
                $len = $bytes[3]
                if ($bytes.Count -ge (5 + $len)) { return $bytes.ToArray() }
            }
        }
        Start-Sleep -Milliseconds 2
    }
    if ($bytes.Count -gt 0) { return $bytes.ToArray() }
    return $null
}

function Show-Frame {
    param([byte[]]$Frame, [string]$Tag = "")
    if ($Frame) {
        $hex = ($Frame | ForEach-Object { $_.ToString("X2") }) -join " "
        Write-Host "$Tag$hex"
    } else {
        Write-Host "${Tag}<no response>"
    }
}

function Send-MotorPing {
    $body = [byte[]](0xAA, 0x03, 0x00, 0x00)
    $crc = Calc-Crc8 $body
    $frame = [byte[]]($body + $crc)
    Send-RawFrame $frame
    Show-Frame $frame "[tx] "
    $r = Read-MotorResponse
    Show-Frame $r "[rx] "
    if ($r -and $r.Count -ge 5 -and $r[4] -eq 0x00) { Write-Host "ping: ok" }
    else { Write-Host "ping: FAIL" }
}

function Read-MotorReg {
    param([byte]$Reg, [byte]$Len = 1)
    $body = [byte[]](0xAA, 0x01, $Reg, $Len)
    $crc = Calc-Crc8 $body
    $frame = [byte[]]($body + $crc)
    Send-RawFrame $frame
    Show-Frame $frame ("[tx] read reg=0x{0:X2} len={1}: " -f $Reg,$Len)
    $r = Read-MotorResponse
    Show-Frame $r "[rx] "
    if ($r -and $r.Count -ge 5) {
        $dlen = $r[3]
        $payload = @()
        if ($dlen -gt 0) { $payload = $r[4..(3+$dlen)] }
        $hex = ($payload | ForEach-Object { $_.ToString("X2") }) -join " "
        Write-Host ("    reg 0x{0:X2} = {1}" -f $Reg,$hex)
        return [byte[]]$payload
    }
    return $null
}

function Write-MotorReg {
    param([byte]$Reg, [byte[]]$Data)
    $body = [byte[]](@(0xAA, 0x02, $Reg, [byte]$Data.Length) + $Data)
    $crc = Calc-Crc8 $body
    $frame = [byte[]]($body + $crc)
    Send-RawFrame $frame
    Show-Frame $frame ("[tx] write reg=0x{0:X2}: " -f $Reg)
    $r = Read-MotorResponse
    Show-Frame $r "[rx] "
    if ($r -and $r.Count -ge 5) {
        if ($r[4] -eq 0x00) { Write-Host "    write: ok" }
        else { Write-Host "    write: ERROR" }
    }
}

function Jog-Motor {
    param(
        [ValidateSet("m1","m2")][string]$Motor = "m1",
        [ValidateRange(1,100)][int]$Duty = 10,
        [ValidateSet("fwd","rev")][string]$Dir = "fwd",
        [int]$Ms = 500
    )
    $base = if ($Motor -eq "m1") { 0x10 } else { 0x20 }
    $dirByte = if ($Dir -eq "rev") { [byte]1 } else { [byte]0 }
    Write-Host ("--- jog {0} duty={1}% {2} for {3}ms ---" -f $Motor,$Duty,$Dir,$Ms)
    Write-MotorReg $base @([byte]1, [byte]$Duty, $dirByte) | Out-Null
    Start-Sleep -Milliseconds $Ms
    Write-MotorReg $base @([byte]0) | Out-Null
    Start-Sleep -Milliseconds 300
}

function Read-MotorStatus {
    Write-Host "=== MotorDriver status dump ==="
    Write-Host "[device id/fw/status/fault/ctrl/i2c/invert]"
    Read-MotorReg 0x00 8 | Out-Null
    Write-Host "[m1 mode/duty/dir/status]"
    Read-MotorReg 0x10 4 | Out-Null
    Write-Host "[m1 encoder count int32 @0x14]"
    Read-MotorReg 0x14 9 | Out-Null
    Write-Host "[m2 mode/duty/dir/status]"
    Read-MotorReg 0x20 4 | Out-Null
    Write-Host "[m2 encoder count int32 @0x24]"
    Read-MotorReg 0x24 9 | Out-Null
    Write-Host "[watchdog/encctrl]"
    Read-MotorReg 0x30 2 | Out-Null
    Write-Host "[target rpm + measured rpm]"
    Read-MotorReg 0x32 8 | Out-Null
    Write-Host "[speed pid kp/ki/kd/max/min]"
    Read-MotorReg 0x3A 5 | Out-Null
}
