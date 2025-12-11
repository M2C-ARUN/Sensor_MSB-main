# ----------------------
# Auto Flash Script NRF52
# ----------------------

# --- Correct SEGGER J-Link path ---
$JLINK = "C:\Program Files\SEGGER\JLink_V818\JLink.exe"

# --- HEX files ---
$SOFTDEVICE = "fallarrest/s132/armgcc/s132_nrf52_7.0.1_softdevice.hex"
$BOOTLOADER = "fallarrest/s132/armgcc/_build/buildbootloader_s132.hex"
$APP_HEX    = "fallarrest/s132/armgcc/_build/nrf52832_xxaa.hex"

Write-Host "Flashing nRF52832..."

# 1. Erase Chip
& $JLINK -device nrf52 -if swd -speed 4000 -autoconnect 1 -CommanderScript erase.jlink

# 2. Flash SoftDevice
& $JLINK -device nrf52 -if swd -speed 4000 -autoconnect 1 -CommanderScript flash_sd.jlink

# 3. Flash Bootloader
& $JLINK -device nrf52 -if swd -speed 4000 -autoconnect 1 -CommanderScript flash_bl.jlink

# 4. Flash Application
& $JLINK -device nrf52 -if swd -speed 4000 -autoconnect 1 -CommanderScript flash_app.jlink

Write-Host "Flash Completed Successfully!"
