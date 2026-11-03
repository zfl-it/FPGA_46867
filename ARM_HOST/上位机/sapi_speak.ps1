# 离线兜底：用 Windows 自带语音（SAPI / System.Speech）朗读一段文本。
# 文本通过临时文件传入，规避命令行编码与转义问题。
param(
    [Parameter(Mandatory = $true)][string]$TextFile,
    [string]$Voice = ""
)

Add-Type -AssemblyName System.Speech
$s = New-Object System.Speech.Synthesis.SpeechSynthesizer
$s.Rate = 0
$s.Volume = 100

if ($Voice) {
    try { $s.SelectVoice($Voice) } catch { }
}

$text = Get-Content -LiteralPath $TextFile -Raw -Encoding UTF8
$s.Speak($text)
$s.Dispose()
