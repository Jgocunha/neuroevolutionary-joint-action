## 🔍 1. Check what USB devices you have

Plug the PTU in, then run:

```bash
dmesg | grep ttyUSB
```

You should see something like:

```
[ 1234.567 ] usb 3-2: ch341-uart converter now attached to ttyUSB0
```

That tells you which `/dev/ttyUSBX` was created when you plugged it in.

If you unplug → replug the adapter, you should see a new `ttyUSB` device appear.
That’s the one to use.


So the device is `/dev/ttyUSB0`.
---

## 3. Connect to the PTU

### : Using `screen`
```bash
sudo apt install screen
screen /dev/ttyUSB0 9600
```
- To see what you type, enable local echo: `Ctrl+A` → `:` → `echo`.
- Exit screen: `Ctrl+A` then `\` (backslash), then `y`.

---


## 🔍 4. Try a manual test with `screen`

```bash
screen /dev/ttyUSB0 9600
```

Then type:

```
POS
```

and press **Enter**.
(If you don’t see your typing, enable echo with `Ctrl+A` → `:` → `echo`.)

---

Ahh 👍 that’s actually a **good sign** — the PTU is alive and talking back.

When it replies with:

```
! Illegal argument
```

…it means it **understood the command structure**, but the argument or formatting wasn’t what it expected.

---

## ✅ Why this happens

On the PTU-46 family:

- **`POS` alone is not valid.**
- You must specify which axis:

  - `PP` → Pan Position
  - `TP` → Tilt Position

- Or ask for **all positions** using `PP` and `TP` separately.

---

## 📡 Correct commands

Try these (each followed by **Enter** / carriage return):

- Query **pan position**:

  ```
  PP
  ```

- Query **tilt position**:

  ```
  TP
  ```

- Move pan to +10° (\~593 units):

  ```
  PP593
  ```

- Move tilt to –5° (\~–297 units):

  ```
  TP-297
  ```

---

## ⚙️ Notes

- The unit uses **counts, not degrees**. For PTU-46:
  ≈ **59.3 counts = 1 degree**.
- So:

  - 593 counts ≈ 10°
  - –297 counts ≈ –5°

- If you send just `PP` or `TP` without a value, it **returns the current position**.
- If you add a number, it **moves to that position**.

---

✅ So instead of `POS`, try:

```
PP
TP
```

and you should see numbers coming back.

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

## sUMMARY

1. sudo dmesg | grep ttyUSB
   [sudo] password for jgocunha:
   [ 138.103994] usb 3-1: pl2303 converter now attached to ttyUSB0
2. screen /dev/ttyUSB0 9600
3. TP-800
