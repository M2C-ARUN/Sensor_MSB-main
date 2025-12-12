# ----------------------
# Interactive Auto Build & Flash Script NRF52
# ----------------------

# Paths
$JLINK        = "C:\Program Files\SEGGER\JLink_V818\JLink.exe"
$PROJECT_PATH = "D:/Testing_code/Sensor_MSB-main/fallarrest/s132/armgcc"

$SOFTDEVICE  = "$PROJECT_PATH/s132_nrf52_7.0.1_softdevice.hex"
$BOOTLOADER  = "$PROJECT_PATH/bootloader_s132.hex"
$APP_HEX     = "$PROJECT_PATH/_build/nrf52832_xxaa.hex"

$ERASE_SCRIPT = "$PROJECT_PATH/erase.jlink"
$FLASH_SD     = "$PROJECT_PATH/flash_sd.jlink"
$FLASH_BL     = "$PROJECT_PATH/flash_bl.jlink"
$FLASH_APP    = "$PROJECT_PATH/flash_app.jlink"

# ---------------------- Helper Functions ----------------------
function Ask-YesNo($message) {
    do {
        $response = Read-Host "$message (y/n)"
    } while ($response -notmatch "^[yYnN]$")
    return $response -match "^[yY]$"
}

function Check-Connection {
    Write-Host -NoNewline "Checking J-Link connection..."

    # Create a temporary script to reset and quit
    $tempScript = New-TemporaryFile
    Set-Content -Path $tempScript -Value "r`nq"

    # Run JLink with the temp script
    $output = & "$JLINK" -device nrf52 -if swd -speed 4000 -autoconnect 1 -CommanderScript $tempScript 2>&1

    Remove-Item $tempScript -Force

    if ($output -match "O\.K\.") {
        Write-Host  " CONNECTION OK "
        return $true
    } else {
        Write-Error " J-Link connection failed. Check USB, power, and SWD connections."
        return $false
    }
}

# Then call it:
if (-not (Check-Connection)) { exit 1 }

function Run-JLinkScript($scriptPath, $desc) {
    Write-Host  -NoNewline "$desc ... "
    $result = & $JLINK -device nrf52 -if swd -speed 4000 -autoconnect 1 -CommanderScript $scriptPath
    if ($LASTEXITCODE -ne 0) {
        Write-Error " $desc failed. Aborting."
        exit 1
    }
    Write-Host "Done"
}


# ---------------------- Step 1: Clean & Build ----------------------

Write-Host -NoNewline " Building project..."
Set-Location $PROJECT_PATH

if (Test-Path "_build/nrf52832_xxaa.hex") {
    Remove-Item "_build/nrf52832_xxaa.hex"
}

$buildResult = & make nrf52832_xxaa
if ($LASTEXITCODE -ne 0) {
    Write-Error " Build failed. Aborting flashing."
    exit 1
}
Write-Host " Build completed!"

# ---------------------- User Selection ----------------------
$doErase    = Ask-YesNo "Erase Chip?"
$doFlashSD  = Ask-YesNo "Flash SoftDevice?"
$doFlashBL  = Ask-YesNo "Flash Bootloader?"
$doFlashApp = Ask-YesNo "Flash Application?"

# ---------------------- Step 2: Erase Chip ----------------------
if ($doErase) {
    Run-JLinkScript $ERASE_SCRIPT "Erasing chip "
}

# ---------------------- Step 3: Flash SoftDevice ----------------------
if ($doFlashSD) {
    Run-JLinkScript $FLASH_SD "Flashing SoftDevice "
}

# ---------------------- Step 4: Flash Bootloader ----------------------
if ($doFlashBL) {
    Run-JLinkScript $FLASH_BL "Flashing Bootloader "
}

# ---------------------- Step 5: Flash Application ----------------------
if ($doFlashApp) {
    Run-JLinkScript $FLASH_APP "Flashing Application "
}

Write-Host " All Steps Completed Successfully!"
