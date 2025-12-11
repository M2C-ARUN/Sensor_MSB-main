# ----------------------
# Interactive Auto Build & Flash Script NRF52
# ----------------------

# Paths
$JLINK       = "C:\Program Files\SEGGER\JLink_V818\JLink.exe"
$PROJECT_PATH = "D:/Testing_code/Sensor_MSB-main/fallarrest/s132/armgcc"

$SOFTDEVICE  = "$PROJECT_PATH/s132_nrf52_7.0.1_softdevice.hex"
$BOOTLOADER  = "$PROJECT_PATH/bootloader_s132.hex"
$APP_HEX     = "$PROJECT_PATH/_build/nrf52832_xxaa.hex"

$ERASE_SCRIPT = "$PROJECT_PATH/erase.jlink"
$FLASH_SD     = "$PROJECT_PATH/flash_sd.jlink"
$FLASH_BL     = "$PROJECT_PATH/flash_bl.jlink"
$FLASH_APP    = "$PROJECT_PATH/flash_app.jlink"

# ---------------------- Helper Function ----------------------
function Ask-YesNo($message) {
    do {
        $response = Read-Host "$message (y/n)"
    } while ($response -notmatch "^[yYnN]$")
    return $response -match "^[yY]$"
}

# ---------------------- Step 0: Clean & Build ----------------------
Write-Host "Building project..."
Set-Location $PROJECT_PATH

if (Test-Path "_build/nrf52832_xxaa.hex") {
    Remove-Item "_build/nrf52832_xxaa.hex"
}

$buildResult = & make nrf52832_xxaa
if ($LASTEXITCODE -ne 0) {
    Write-Error "Build failed. Aborting flashing."
    exit 1
}
Write-Host "✅ Build completed!"

# ---------------------- User Selection ----------------------
$doErase      = Ask-YesNo "Erase Chip?"
$doFlashSD    = Ask-YesNo "Flash SoftDevice?"
$doFlashBL    = Ask-YesNo "Flash Bootloader?"
$doFlashApp   = Ask-YesNo "Flash Application?"

# ---------------------- Step 1: Erase Chip ----------------------
if ($doErase) {
    Write-Host "Erasing chip..."
    $eraseResult = & $JLINK -device nrf52 -if swd -speed 4000 -autoconnect 1 -CommanderScript $ERASE_SCRIPT
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Erase failed. Aborting."
        exit 1
    }
}

# ---------------------- Step 2: Flash SoftDevice ----------------------
if ($doFlashSD) {
    Write-Host "Flashing SoftDevice..."
    $sdResult = & $JLINK -device nrf52 -if swd -speed 4000 -autoconnect 1 -CommanderScript $FLASH_SD
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Flashing SoftDevice failed. Aborting."
        exit 1
    }
}

# ---------------------- Step 3: Flash Bootloader ----------------------
if ($doFlashBL) {
    Write-Host "Flashing Bootloader..."
    $blResult = & $JLINK -device nrf52 -if swd -speed 4000 -autoconnect 1 -CommanderScript $FLASH_BL
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Flashing Bootloader failed. Aborting."
        exit 1
    }
}

# ---------------------- Step 4: Flash Application ----------------------
if ($doFlashApp) {
    Write-Host "Flashing Application..."
    $appResult = & $JLINK -device nrf52 -if swd -speed 4000 -autoconnect 1 -CommanderScript $FLASH_APP
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Flashing Application failed. Aborting."
        exit 1
    }
}

Write-Host "✅ Build & Flash Process Completed Successfully!"
