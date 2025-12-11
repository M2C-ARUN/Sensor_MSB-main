# ----------------------
# Auto Build & Flash Script NRF52
# ----------------------

$JLINK = "C:\Program Files\SEGGER\JLink_V818\JLink.exe"
$PROJECT_PATH = "D:/Testing_code/Sensor_MSB-main/fallarrest/s132/armgcc"

$SOFTDEVICE = "fallarrest/s132/armgcc/s132_nrf52_7.0.1_softdevice.hex"

$BOOTLOADER = "D:/Testing_code/Sensor_MSB-main/fallarrest/s132/armgcc/bootloader_s132.hex"
$APP_HEX    = "fallarrest/s132/armgcc/_build/nrf52832_xxaa.hex"

$ERASE_SCRIPT = "$PROJECT_PATH/erase.jlink"
$FLASH_SD    = "$PROJECT_PATH/flash_sd.jlink"
$FLASH_BL    = "$PROJECT_PATH/flash_bl.jlink"
$FLASH_APP   = "$PROJECT_PATH/flash_app.jlink"

# --- Step 0: Clean & Build ---
Write-Host "Building project..."
cd $PROJECT_PATH

if (Test-Path "_build/nrf52832_xxaa.hex") {
    Remove-Item "_build/nrf52832_xxaa.hex"
}
# Build application
& make nrf52832_xxaa
Write-Host "Build completed!"

# 1. Erase Chip
& $JLINK -device nrf52 -if swd -speed 4000 -autoconnect 1 -CommanderScript erase.jlink

# 2. Flash SoftDevice
& $JLINK -device nrf52 -if swd -speed 4000 -autoconnect 1 -CommanderScript flash_sd.jlink

# 3. Flash Bootloader
& $JLINK -device nrf52 -if swd -speed 4000 -autoconnect 1 -CommanderScript flash_bl.jlink

# 4. Flash Application
& $JLINK -device nrf52 -if swd -speed 4000 -autoconnect 1 -CommanderScript flash_app.jlink

Write-Host "Flash Completed Successfully!"
