$listener = New-Object System.Net.HttpListener
$listener.Prefixes.Add("http://localhost:8080/")
$listener.Start()
Write-Host "Serving on http://localhost:8080/ (Ctrl+C to stop)"
$base = $PSScriptRoot
while ($listener.IsListening) {
    $ctx = $listener.GetContext()
    $reqPath = $ctx.Request.Url.AbsolutePath.TrimStart('/')
    if ([string]::IsNullOrEmpty($reqPath)) { $reqPath = 'index.html' }
    $filePath = Join-Path $base $reqPath
    Write-Host "Request: $($ctx.Request.Url.AbsolutePath) -> $filePath"
    if (Test-Path -LiteralPath $filePath -PathType Leaf) {
        $content = [System.IO.File]::ReadAllBytes($filePath)
        # Development tool assets must reflect the checked-out source after a
        # refresh; stale JavaScript can otherwise hide firmware/tool changes.
        $ctx.Response.Headers['Cache-Control'] = 'no-store, no-cache, must-revalidate'
        $ctx.Response.Headers['Pragma'] = 'no-cache'
        $ctx.Response.Headers['Expires'] = '0'
        $ctx.Response.ContentLength64 = $content.Length
        if ($reqPath -match '\.js$') { $ctx.Response.ContentType = 'application/javascript; charset=utf-8' }
        elseif ($reqPath -match '\.html$') { $ctx.Response.ContentType = 'text/html; charset=utf-8' }
        elseif ($reqPath -match '\.css$') { $ctx.Response.ContentType = 'text/css; charset=utf-8' }
        elseif ($reqPath -match '\.json$') { $ctx.Response.ContentType = 'application/json; charset=utf-8' }
        elseif ($reqPath -match '\.svg$') { $ctx.Response.ContentType = 'image/svg+xml' }
        else { $ctx.Response.ContentType = 'application/octet-stream' }
        $ctx.Response.OutputStream.Write($content, 0, $content.Length)
    } else {
        Write-Host "  -> 404"
        $ctx.Response.StatusCode = 404
    }
    $ctx.Response.Close()
}
