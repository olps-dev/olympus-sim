$client = New-Object System.Net.Sockets.TcpClient
try {
    Write-Host "🔌 Connecting to Project Olympus Demo on localhost:3333..."
    $client.Connect('localhost', 3333)
    Write-Host "✅ Connected! Reading UART output..."
    Write-Host ("=" * 50)
    
    $stream = $client.GetStream()
    $reader = New-Object System.IO.StreamReader($stream)
    
    # Read for 30 seconds or until we get enough data
    $startTime = Get-Date
    $lineCount = 0
    
    while (((Get-Date) - $startTime).TotalSeconds -lt 30 -and $lineCount -lt 50) {
        $line = $reader.ReadLine()
        if ($line) {
            Write-Host "📤 $line"
            $lineCount++
        } else {
            Start-Sleep -Milliseconds 100
        }
    }
    
    Write-Host ("=" * 50)
    Write-Host "🎯 Demo completed! Received $lineCount lines of output."
    
} catch {
    Write-Host "❌ Connection failed: $($_.Exception.Message)"
    Write-Host "💡 Make sure the demo is running with: make demo"
} finally {
    if ($client) {
        $client.Close()
    }
} 