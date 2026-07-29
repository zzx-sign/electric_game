$files = @(
    'e:\FreeTros Task\电电的赛\electric_game\Core\Src\pid.h',
    'e:\FreeTros Task\电电的赛\electric_game\Core\Src\app.h',
    'e:\FreeTros Task\电电的赛\electric_game\Core\Src\motor.h',
    'e:\FreeTros Task\电电的赛\electric_game\Core\Src\motor.c',
    'e:\FreeTros Task\电电的赛\electric_game\Core\Src\app.c',
    'e:\FreeTros Task\电电的赛\electric_game\Core\Src\pid.c'
)
foreach ($file in $files) {
    $bytes = [System.IO.File]::ReadAllBytes($file)
    $len = $bytes.Length
    $nl = "`r`n"
    $endsWithNewline = $false
    if ($len -ge 2 -and $bytes[$len-2] -eq 13 -and $bytes[$len-1] -eq 10) { $endsWithNewline = $true }
    elseif ($len -ge 1 -and $bytes[$len-1] -eq 10) { $endsWithNewline = $true }
    if (-not $endsWithNewline) {
        $append = [System.Text.Encoding]::UTF8.GetBytes($nl)
        [System.IO.File]::AppendAllText($file, $nl)
        Write-Host "Patched: $file"
    } else {
        Write-Host "OK:      $file"
    }
}