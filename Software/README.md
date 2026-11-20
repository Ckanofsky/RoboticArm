# Bridge firmware (XIAO STM32C5)

Runs on the Seeed Studio XIAO STM32C5. It talks to the CubeMars motors over CAN
and (later) to the PC app over the USB-C serial port.

## Build and upload
1. In VS Code, open **this `Software` folder** (File > Open Folder). PlatformIO only
   finds `platformio.ini` when it is in the folder you opened.
2. Build: PlatformIO checkmark button, or `pio run`.
3. Upload: PlatformIO arrow button, or `pio run -t upload`.
   If that fails, double-press Reset on the XIAO, wait for the **XIAOC5BOOT** drive,
   and copy `.pio/build/seeed-xiao-stm32c5/zephyr/zephyr.uf2` onto it.
4. Open the PlatformIO Serial Monitor (plug icon) to see output.

## First motor test
1. With the R-Link + CubeMars Tool V3.0: set the motor's CAN ID to 1 and turn on
   timed CAN feedback (e.g. 100 Hz).
2. Wire: XIAO CAN_H -> motor CAN_H, CAN_L -> CAN_L, XIAO GND -> 24 V supply negative.
   The XIAO has a 120 ohm terminator on board; put one 120 ohm resistor at the motor end.
3. Power the motor (bench supply at 24 V, low current limit), upload, open the monitor.
   You should see lines like `id=0x... pos=12.3 deg  spd=0 erpm  cur=0.05 A ...`.
   The last two hex digits of `id` are the motor's CAN ID.
4. Only with the motor clamped down: set `TEST_MOVE` to 1 in `src/main.c` to make
   it turn +10 degrees slowly and come back.
