# ---------------- CONFIG ----------------
$MAKE_CMD = "make"
$BUILD_DIR = ".\_build"
$TARGET = "nrf52832_xxaa"
$OUT_FILE = "$BUILD_DIR\$TARGET.out"
$HEX_FILE = "$BUILD_DIR\$TARGET.hex"

# ---------------- BUILD ----------------
Write-Host "Building project..."
& $MAKE_CMD clean
& $MAKE_CMD all

# Check .out file
If (!(Test-Path $OUT_FILE)) {
    Write-Host "Error: Output file $OUT_FILE not found! Build failed."
    exit 1
}

# ---------------- CONVERT ----------------
Write-Host "Converting .out to .hex..."
& "C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\bin\arm-none-eabi-objcopy.exe" -O ihex $OUT_FILE $HEX_FILE

If (!(Test-Path $HEX_FILE)) {
    Write-Host "Error: HEX file $HEX_FILE not found! Conversion failed."
    exit 1
}

# ---------------- DETECT DEBUGGER ----------------
Write-Host "Detecting connected J-Link debugger..."
$ids = & nrfjprog --ids
If (!$ids) {
    Write-Host "Error: No debugger found! Connect your J-Link and try again."
    exit 1
}

$DEBUG_SN = $ids[0].Trim()
Write-Host "Using debugger serial number: $DEBUG_SN"

# ---------------- FLASH ----------------
Write-Host "Flashing $HEX_FILE..."
nrfjprog --eraseall --sn $DEBUG_SN
nrfjprog --program $HEX_FILE --verify --sn $DEBUG_SN
nrfjprog --reset --sn $DEBUG_SN

Write-Host "✅ Flashed $HEX_FILE successfully!"
