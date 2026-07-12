$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
$header = Get-Content -Raw -LiteralPath (Join-Path $root 'project\code\Motor.h')
$source = Get-Content -Raw -LiteralPath (Join-Path $root 'project\code\Motor.c')

$requiredHeader = @(
    '#define\s+MOTOR_LEFT_EN_PWM\s+\(\s*PWMB_CH3_P76\s*\)',
    '#define\s+MOTOR_LEFT_PH\s+\(\s*IO_P75\s*\)',
    '#define\s+MOTOR_LEFT_NSLEEP\s+\(\s*IO_P74\s*\)',
    '#define\s+MOTOR_RIGHT_EN_PWM\s+\(\s*PWMD_CH1_P50\s*\)',
    '#define\s+MOTOR_RIGHT_PH\s+\(\s*IO_P77\s*\)',
    '#define\s+MOTOR_RIGHT_NSLEEP\s+\(\s*IO_P51\s*\)',
    '#define\s+PWM_L\s+\(\s*MOTOR_LEFT_EN_PWM\s*\)',
    '#define\s+PWM_R\s+\(\s*MOTOR_RIGHT_EN_PWM\s*\)'
)

foreach ($pattern in $requiredHeader) {
    if ($header -notmatch $pattern) {
        throw "Motor.h is missing required mapping: $pattern"
    }
}

foreach ($pattern in @('gpio_init\(MOTOR_LEFT_NSLEEP,\s*GPO,\s*GPIO_LOW',
                         'gpio_init\(MOTOR_RIGHT_NSLEEP,\s*GPO,\s*GPIO_LOW',
                         'gpio_set_level\(MOTOR_LEFT_NSLEEP,\s*GPIO_HIGH',
                         'gpio_set_level\(MOTOR_RIGHT_NSLEEP,\s*GPIO_HIGH')) {
    if ($source -notmatch $pattern) {
        throw "Motor.c is missing required DRV8701 startup behavior: $pattern"
    }
}

foreach ($pattern in @('TIM17_ENCODER',
                         'TIM17_ENCODER_CH1_P80',
                         'IO_P44',
                         'TIM18_ENCODER',
                         'TIM18_ENCODER_CH1_P90',
                         'IO_P46')) {
    if ($header -notmatch $pattern) {
        throw "Motor.h is missing required encoder resource: $pattern"
    }
}

foreach ($pattern in @('T17H', 'T17L', 'T18H', 'T18L',
                         'gpio_get_level\(ENCODER_RIGHT_DIR_PIN\)',
                         'gpio_get_level\(ENCODER_LEFT_DIR_PIN\)')) {
    if ($source -notmatch $pattern) {
        throw "Motor.c is missing required TIM17/TIM18 encoder handling: $pattern"
    }
}

$laserSource = Get-Content -Raw -LiteralPath (Join-Path $root 'project\code\laser.c')

# Verify laser pins don't conflict with encoder direction pins
if ($laserSource -match 'IO_P44') {
    throw 'laser.c MUST NOT use IO_P44: it is the encoder right DIR pin (see Motor.h).'
}
if ($laserSource -match 'IO_P46') {
    throw 'laser.c MUST NOT use IO_P46: it is the encoder left DIR pin (see Motor.h).'
}

$mainSource = Get-Content -Raw -LiteralPath (Join-Path $root 'project\user\main.c')
$initHeader = Get-Content -Raw -LiteralPath (Join-Path $root 'project\code\Init.h')
$initSource = Get-Content -Raw -LiteralPath (Join-Path $root 'project\code\Init.c')
if ($mainSource -notmatch 'uint8\s+ControlFlag\s*=\s*1') {
    throw 'Race firmware must default ControlFlag to image-processing mode.'
}
if ($initHeader -notmatch '#define\s+APP_RACE_MODE\s+1') {
    throw 'Init.h is missing the race-mode compile-time switch.'
}
if ($initSource -notmatch '#if\s*!APP_RACE_MODE') {
    throw 'Init.c does not compile WiFi initialization out of race mode.'
}

Write-Output 'Hardware mapping static contract passed.'
