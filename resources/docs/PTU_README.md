# PTU-46-17.5 Quick Control Guide (Ubuntu 22.04 + USB)

This guide explains how to connect to and control the Directed Perception PTU-46-17.5 Pan-Tilt Unit on Ubuntu 22.04 using a USB-to-RS232 adapter.

---

## 1. Identify the USB Port

Plug in the PTU adapter, then check which `/dev/ttyUSBX` was created:

```bash
dmesg | grep ttyUSB
```

Example output:
```
[ 1234.567 ] usb 3-2: ch341-uart converter now attached to ttyUSB0
```

So the device is `/dev/ttyUSB0`.

---

## 2. Check Permissions

List devices:
```bash
ls -l /dev/ttyUSB*
```

If your user is not in the `dialout` group:
```bash
sudo usermod -a -G dialout $USER
```
Then log out and back in.

---

## 3. Connect to the PTU

### Option A: Using `screen`
```bash
sudo apt install screen
screen /dev/ttyUSB0 9600
```
- To see what you type, enable local echo: `Ctrl+A` → `:` → `echo`.
- Exit screen: `Ctrl+A` then `\` (backslash), then `y`.

### Option B: Using `minicom`
```bash
sudo apt install minicom
minicom -D /dev/ttyUSB0 -b 9600
```
Enable local echo inside minicom with `Ctrl+A` then `E`.

---

## 4. Sending Commands

⚠️ **Important**: Commands must end with a carriage return (`\r`).  
Press **Enter** after typing each command.

### Query Position
- Pan position:
  ```
  PP
  ```
- Tilt position:
  ```
  TP
  ```

### Move to Position
The PTU uses **counts**, not degrees.  
For PTU-46: **~59.3 counts = 1 degree**.

- Move pan to +10°:
  ```
  PP 593
  ```
- Move tilt to –5°:
  ```
  TP -297
  ```

### Other Useful Commands
- Set Pan Speed (counts/sec):
  ```
  PS 500
  ```
- Set Tilt Speed (counts/sec):
  ```
  TS 500
  ```
- Reset (re-home):
  ```
  R
  ```

---

## 5. One-Shot Commands from Shell

You can also send commands directly without `screen`/`minicom`:

```bash
stty -F /dev/ttyUSB0 9600 cs8 -cstopb -parenb
echo -ne "PP\r" > /dev/ttyUSB0
cat < /dev/ttyUSB0
```

---

## 6. Notes
- Default serial settings: **9600 8N1**.
- Commands are **case-sensitive**.
- Always send `\r` at the end of commands.
- If you see `! Illegal argument`, it means the command format was wrong.

---

## Cheat Sheet

| Action             | Command Example |
|--------------------|-----------------|
| Pan position query | `PP`            |
| Tilt position query| `TP`            |
| Pan to +30°        | `PP 1780`       |
| Tilt to -10°       | `TP -593`       |
| Set pan speed      | `PS 1000`       |
| Set tilt speed     | `TS 1000`       |
| Reset unit         | `R`             |

---

Happy tilting! 🎥
