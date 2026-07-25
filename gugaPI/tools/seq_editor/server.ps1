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
        $ctx.Response.ContentLength64 = $content.Length
        if ($reqPath -match '\.js$') { $ctx.Response.ContentType = 'application/javascript' }
        elseif ($reqPath -match '\.html$') { $ctx.Response.ContentType = 'text/html; charset=utf-8' }
        else { $ctx.Response.ContentType = 'application/octet-stream' }
        $ctx.Response.OutputStream.Write($content, 0, $content.Length)
    } else {
        Write-Host "  -> 404"
        $ctx.Response.StatusCode = 404
    }
    $ctx.Response.Close()
}
