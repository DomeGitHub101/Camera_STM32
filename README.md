# OV7670 AprilTag Reader on NUCLEO-F767ZI

กล้อง OV7670 (รุ่นไม่มี FIFO) ต่อกับบอร์ด NUCLEO-F767ZI อ่าน **AprilTag (tag36h11)** และแปลงเป็นคำสั่งหุ่นยนต์ **บน STM32 ทั้งหมด** แล้วส่งผลออกทาง Tera Term ตามรอบเวลาที่ตั้งได้
สคริปต์ Python ใช้ **แสดงภาพจากกล้องอย่างเดียว** (วาดกรอบตามตำแหน่ง tag ที่ STM32 ส่งมา ไม่ได้ถอดรหัสเอง)

```
OV7670 ──DCMI+DMA──▶ STM32F767 ──▶ หา AprilTag ──▶ ตัดสินคำสั่ง ──UART──▶ Tera Term
   ▲                                                              └──▶ Python viewer (ภาพ)
   └── XCLK 13.5 MHz (TIM2) + ตั้งค่าผ่าน SCCB (I2C2)
```

---

## 1. อุปกรณ์

| อุปกรณ์ | จำนวน | หมายเหตุ |
|---|---|---|
| NUCLEO-F767ZI | 1 | |
| กล้อง OV7670 แบบไม่มี FIFO (18 ขา) | 1 | |
| ตัวต้านทาน 4.2 kΩ | 2 | pull-up ของ SIOD/SIOC (ค่าที่ใช้จริงในชุดนี้; 2.2–10 kΩ ใช้ได้) |
| สาย jumper ตัวเมีย–ตัวผู้ | ~20 | ยิ่งสั้นยิ่งดี |
| ตัวเก็บประจุ ceramic 100–470 pF | 1 | *ไม่บังคับ* กรอง glitch ที่ VSYNC |
| สาย USB (Micro-B) | 1 | เสียบช่อง ST-LINK (CN1) |
| จอ 2.8" TFT SPI 240×320 V1.2 (ILI9341 + ทัช XPT2046) | 1 | หน้าเลือกโต๊ะ / หน้าแมว (โหมด `APP_DELIVERY`) |
| Servo (SG90 หรือเทียบเท่า) | 1 | หมุนกล้องซ้าย-ขวา (pan) — ใช้ไฟ 5V แยก |

---

## 2. Pinout และการต่อสาย

> ขา STM32 ตรงกับที่ตั้งไว้ใน `Camera_setting_with_LCD.ioc`
> ตำแหน่งบน header อ้างอิงคู่มือ NUCLEO-144 (UM1974) — **เทียบกับบอร์ดจริงอีกครั้งก่อนจ่ายไฟ** โดยเฉพาะ 3V3 / 5V

### 2.1 ตารางต่อสาย OV7670 → NUCLEO-F767ZI

| ขากล้อง | ขา STM32 | ตำแหน่งบนบอร์ด | Arduino | หน้าที่ |
|---|---|---|---|---|
| **3V3** | +3V3 | CN8 ขา 7 | 3V3 | ไฟเลี้ยง — **ห้ามต่อ 5V** |
| **GND** | GND | CN8 ขา 11 | GND | กราวด์ |
| **SIOC** (SCL) | **PF1** | CN9 ขา 19 | D69 | SCCB clock (I2C2_SCL) |
| **SIOD** (SDA) | **PF0** | CN9 ขา 21 | D68 | SCCB data (I2C2_SDA) |
| **XCLK** | **PA5** | CN7 ขา 10 | D13 | clock 13.5 MHz ให้กล้อง (TIM2_CH1) |
| **PCLK** | **PA6** | CN7 ขา 12 | D12 | pixel clock (DCMI_PIXCLK) |
| **HREF** (HS) | **PA4** | CN7 ขา 17 | D24 | ข้อมูลบรรทัดถูกต้อง (DCMI_HSYNC) |
| **VSYNC** (VS) | **PG9** | CN10 ขา 16 | D0 | เริ่มเฟรม (DCMI_VSYNC) |
| **D0** | **PC6** | CN7 ขา 1 | D16 | DCMI_D0 |
| **D1** | **PC7** | CN7 ขา 11 | D21 | DCMI_D1 |
| **D2** | **PE0** | CN10 ขา 33 | D34 | DCMI_D2 |
| **D3** | **PC9** | CN8 ขา 4 | D44 | DCMI_D3 |
| **D4** | **PC11** | CN8 ขา 8 | D46 | DCMI_D4 |
| **D5** | **PD3** | CN9 ขา 10 | D55 | DCMI_D5 |
| **D6** | **PB8** | CN7 ขา 2 | D15 | DCMI_D6 |
| **D7** | **PB9** | CN7 ขา 4 | D14 | DCMI_D7 |
| **RESET** | — | ต่อร่วมกับ 3V3 | — | ต้องเป็น HIGH ไม่อย่างนั้นกล้องค้างใน reset |
| **PWDN** | — | CN8 ขา 13 (GND) | — | ต้องเป็น LOW ไม่อย่างนั้นกล้องอยู่ในโหมด power-down |

### 2.2 อุปกรณ์เสริมที่ต้องใส่

```
            3V3
             │
       ┌─────┴─────┐
     4.2kΩ       4.2kΩ
       │           │
SIOD ──┴── PF0     │
SIOC ──────────────┴── PF1
```

| ชิ้นส่วน | ต่อระหว่าง | จำเป็น? |
|---|---|---|
| 4.2 kΩ | SIOD ↔ 3V3 | **จำเป็น** — ไม่ใส่แล้ว SCCB ตอบ ACK ไม่สม่ำเสมอ / bus ค้าง |
| 4.2 kΩ | SIOC ↔ 3V3 | **จำเป็น** |
| สาย GND เพิ่ม 2–3 เส้น | GND กล้อง ↔ GND บอร์ด (เช่น CN8 ขา 13, CN10 ขา 17) | **แนะนำมาก** — ลด ground bounce ที่ทำให้ VSYNC มี glitch |
| 100–470 pF | VSYNC ↔ GND (ใกล้ PG9) | ไม่บังคับ — ใส่ถ้ายังเห็น glitch ที่ VSYNC |

### 2.3 ข้อควรระวังในการเดินสาย

- **ไฟเลี้ยงกล้องต้องเป็น 3.3V เท่านั้น** — ระวังขา CN8 ขา 9 ที่เป็น 5V อยู่ติดกัน
- **RESET → 3V3 และ PWDN → GND ห้ามปล่อยลอย** (สาเหตุอันดับ 1 ที่กล้องไม่ตอบ)
- สาย **XCLK และ PCLK** เป็นสัญญาณความถี่สูง ใช้สายสั้นที่สุด (< 15–20 ซม.)
- เดินสาย **VSYNC แยกจาก XCLK/PCLK** อย่ามัดรวมเป็นแพ
- USART3 (PD8/PD9) วิ่งผ่าน USB ของ ST-LINK อยู่แล้ว **ไม่ต้องต่อสาย UART เพิ่ม**

### 2.4 ขาที่ใช้งานภายในบอร์ด (ไม่ต้องต่อ)

| ขา | หน้าที่ |
|---|---|
| PD8 / PD9 | USART3 TX/RX → ST-LINK Virtual COM Port |
| PA13 / PA14 | SWD (debug/flash ผ่าน ST-LINK) |

### 2.5 จอ LCD ILI9341 2.8" + ทัช XPT2046 → NUCLEO-F767ZI

ขาและไลบรารีตามเอกสารวิชา Lec11 (ไลบรารี `ILI9341_STM32_Driver`, `ILI9341_GFX`, `ILI9341_Touchscreen`)
จอใช้ SPI5 (Full-Duplex Master, 8 bit, CPOL Low, CPHA 1 Edge, prescaler 4) ส่วนทัชอ่านแบบ bit-bang

| ขาบนโมดูลจอ | ขา STM32 | ตำแหน่งบนบอร์ด | หน้าที่ |
|---|---|---|---|
| **VCC** | +3V3 หรือ +5V | เช่น CN8 ขา 7 (3V3) | ไฟเลี้ยง — โมดูล V1.2 มี regulator รับได้ทั้งสองแบบ (สัญญาณเป็น 3.3V) |
| **GND** | GND | เช่น CN9 ขา 12 | กราวด์ |
| **CS** | **PC8** | CN8 ขา 2 | LCD chip select |
| **RESET** | **PC10** | CN8 ขา 6 | LCD reset |
| **DC** (D/C, RS) | **PC12** | CN8 ขา 10 | data / command — **ไม่ใช่ PC9** (PC9 = DCMI_D3 ของกล้อง) |
| **SDI** (MOSI) | **PF9** | CN9 ขา 28 | SPI5_MOSI |
| **SCK** | **PF7** | CN9 ขา 26 | SPI5_SCK |
| **LED** | +3V3 | ร่วมกับ VCC | ไฟพื้นหลัง (ต่อค้างไว้) |
| **SDO** (MISO) | **PF8** | CN9 ขา 24 | SPI5_MISO |
| **T_CLK** | **PE3** | CN9 ขา 22 | ทัช clock (bit-bang) |
| **T_CS** | **PE6** | CN9 ขา 20 | ทัช chip select |
| **T_DIN** | **PE5** | CN9 ขา 18 | ทัช data in (MOSI) |
| **T_DO** | **PE4** | CN9 ขา 16 | ทัช data out (MISO) |
| **T_IRQ** | **PE2** | CN9 ขา 14 | แตะจอ = LOW → EXTI2 (falling, pull-up) |

- จอวางแนวนอน 320×240 (`ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1)`)
- ภาพบนจอเพี้ยน → เพิ่ม prescaler ของ SPI5 (ช้าลง) ใน CubeMX
- แตะแล้วตำแหน่งไม่ตรง → ปรับ `TP_ADC_X_LEFT/RIGHT`, `TP_ADC_Y_TOP/BOTTOM` ใน `Core/Src/cat_ui.c`
  (บรรทัด `TOUCH x=.. y=.. (adc a,b)` ใน Tera Term บอกค่าที่อ่านได้)

### 2.6 Servo หมุนกล้อง (pan)

| ขา servo | ต่อกับ | หมายเหตุ |
|---|---|---|
| สัญญาณ (ส้ม/เหลือง) | **PD14** (CN7 ขา 16) | TIM4_CH3, 50 Hz, 500–2500 µs = −90°..+90° |
| V+ (แดง) | **5V จากแหล่งจ่ายแยก** | อย่าดึงไฟจากบอร์ด — servo กระชากกระแสจน ST-LINK หลุด |
| GND (น้ำตาล/ดำ) | GND แหล่งจ่าย **และ** GND บอร์ด | ต้องต่อกราวด์ร่วม |

ยังไม่ต่อ servo ให้ตั้ง `PAN_SERVO_ENABLED 0` ใน `Core/Src/delivery.c` (กล้องมองตรงไปข้างหน้า ไม่กวาด)
หมุนผิดทาง → `PAN_SERVO_DIR -1`

### 2.7 บอร์ดล่าง (มอเตอร์ + ToF) — USART2

| สัญญาณ | บอร์ดบน | บอร์ดล่าง |
|---|---|---|
| UART บน → ล่าง | PD5 (USART2_TX, CN9 ขา 6) | PD6 (USART2_RX) |
| UART ล่าง → บน | PD6 (USART2_RX, CN9 ขา 4) | PD5 (USART2_TX) |
| GND | GND | GND (ต่อร่วม) |

115200 8N1 · บน → ล่าง `$M,left,right,mode*XX` ทุก 100 ms · ล่าง → บน `$O,level,dist_mm*XX` ทุก 200 ms
(checksum = XOR ทุกตัวอักษรระหว่าง `$` กับ `*`) — ยังไม่ได้เปิด USART2 ในเฟิร์มแวร์ตัวนี้

---

## 3. Setup ซอฟต์แวร์

### 3.1 สิ่งที่ต้องติดตั้ง

| โปรแกรม | ใช้ทำ |
|---|---|
| STM32CubeIDE (ทดสอบกับ 2.2.0, STM32Cube FW_F7 V1.17.4) | build + flash firmware |
| Tera Term 5 | ดูผลคำสั่ง |
| Python 3.10+ (ไม่บังคับ) | ดูภาพจากกล้อง |

### 3.2 Build และ Flash

1. เปิด STM32CubeIDE → **File → Import → Existing Projects into Workspace** → เลือกโฟลเดอร์โปรเจกต์นี้
2. **Project → Build Project** (Ctrl+B) — ต้องขึ้น `Build Finished. 0 errors`
3. เสียบ USB ที่ช่อง ST-LINK แล้วกด **Run ▶** — ต้องขึ้น `Download verified successfully`

> ⚠️ **ไม่ต้อง generate code ใหม่จาก CubeMX** ถ้าจำเป็นต้อง generate ให้ตั้งใน CubeMX ก่อน:
> **PA5 → GPIO output speed = Very High** (XCLK 13.5 MHz) — ค่านี้ถูกตั้งไว้ใน `tim.c` แต่ไม่ได้อยู่ใน `.ioc`
> code ที่เขียนเพิ่มทั้งหมดอยู่ในส่วน `USER CODE` จึงไม่หายตอน generate

### 3.3 ดูผลใน Tera Term 5

1. **File → New connection → Serial** → เลือกพอร์ต `STMicroelectronics STLink Virtual COM Port (COMx)`
2. **Setup → Serial port** → Speed: **พิมพ์ `2000000`** ลงไปเอง (ไม่มีในรายการ) → Data 8 bit, Parity none, Stop 1 bit, Flow control none
3. **Setup → Terminal** → New-line Receive: **CR** (ค่าเริ่มต้น) — ถ้าบรรทัดเว้นว่างซ้อน ให้ตรวจค่านี้
4. กดปุ่ม **RESET (สีดำ)** บนบอร์ด

ผลที่ได้ (ตัวอย่างรูปแบบ):

```
=== OV7670 AprilTag reader (tag36h11, detection on the STM32) ===
camera: OK (PID 0x76 VER 0x73), grey image, detection on 320x240
tag ID -> command: 0 STOP, 1 FORWARD, 2 TURN_LEFT, 3 TURN_RIGHT, 4 BACKWARD
output delay 500 ms  (keys: + slower, - faster, h help)
[     5.50 s] CMD: FORWARD       | tag ID 1 at (162,118) 44px rot 0 err 0 | detect 25.0 ms, 15.0 fps
[     6.00 s] CMD: STOP (no tag) | no tag | detect 18.0 ms, 16.0 fps
```

| ส่วนของบรรทัด | ความหมาย |
|---|---|
| `[ 5.50 s]` | เวลานับจากเปิดเครื่อง |
| `CMD: FORWARD` | คำสั่งปัจจุบันของหุ่นยนต์ |
| `tag ID 1 at (162,118)` | tag ที่ใหญ่ที่สุดในภาพ และตำแหน่งกึ่งกลาง (พิกเซล) |
| `44px` | ขนาดด้านของกรอบดำ (ใหญ่ = ใกล้) |
| `rot 0` | tag หมุนตามเข็มนาฬิกา 0/90/180/270 องศา |
| `err 0` | จำนวน bit ที่แก้ไข (0–2) |
| `+more` | เห็นมากกว่า 1 tag |
| `detect 25.0 ms` | เวลาที่ STM32 ใช้หา tag ต่อเฟรม |
| `15.0 fps` | จำนวนเฟรมที่ประมวลผลได้ต่อวินาที |

**ปุ่มใน Tera Term**

| ปุ่ม | ผล |
|---|---|
| `+` | เพิ่ม delay ระหว่างบรรทัดผล 100 ms (สูงสุด 10000 ms) |
| `-` | ลด delay 100 ms (ต่ำสุด 100 ms) |
| `h` | แสดงวิธีใช้ |

> ห้ามกด `v` ใน Tera Term — เป็นคำสั่งเปิด video (ข้อมูล binary) สำหรับ Python ถ้าเผลอกด ให้กด `t` เพื่อกลับเป็นข้อความ

### 3.4 ดูภาพด้วย Python (ไม่บังคับ)

```
pip install pyserial numpy opencv-python
python tools/ov7670_viewer.py            # หาพอร์ต ST-LINK เอง
python tools/ov7670_viewer.py COM3       # หรือระบุพอร์ต
```

- **ปิด Tera Term ก่อน** — ทั้งสองโปรแกรมใช้ COM port เดียวกัน เปิดพร้อมกันไม่ได้
- viewer ส่ง `v` ขอภาพจากบอร์ดเอง และส่ง `t` ตอนปิด (เปิด Tera Term ต่อได้ทันที)
- หน้าต่างแสดงภาพ 160×120 ขยาย 4 เท่า + กรอบรอบ tag + คำสั่งที่ STM32 ตัดสิน
- ข้อความผลเหมือนใน Tera Term พิมพ์ใน console
- ปุ่ม: `s` บันทึกภาพ PNG, `q` ปิด

---

## 4. AprilTag ที่ใช้

- Family: **tag36h11** เท่านั้น (รูปจากอินเทอร์เน็ตหลายรูปเป็น tagStandard41h12 ซึ่งอ่านไม่ได้)
- รูปพร้อมใช้: `tools/tags/tag36h11_id1.png` … `tag36h11_id4.png` (พิมพ์หรือเปิดบนมือถือ)
- ต้องมี **ขอบขาวรอบ tag** อย่างน้อย 1 ช่อง

| Tag ID | คำสั่ง |
|---|---|
| 0 | STOP |
| 1 | FORWARD |
| 2 | TURN_LEFT |
| 3 | TURN_RIGHT |
| 4 | BACKWARD |
| อื่น ๆ | แสดงผล แต่ไม่เปลี่ยนคำสั่ง |

เปลี่ยนคำสั่งได้ที่ตาราง `tag_cmd[]` ใน `Core/Src/main.c`

**ระยะอ่าน (ประมาณ):** กรอบดำของ tag ต้องกว้าง ≥ 24 px ในภาพ

| ความละเอียดที่ใช้หา tag | tag 10 ซม. | tag 15 ซม. |
|---|---|---|
| 320×240 (`DET_SCALE 1`, ค่าเริ่มต้น) | ~1.4 ม. | ~2 ม. |
| 160×120 (`DET_SCALE 2`) | ~0.7 ม. | ~1 ม. |

> เลนส์ OV7670 ปรับโฟกัสด้วยการหมุน — ถ้าอ่านใกล้ ๆ ไม่ได้ ลองหมุนเลนส์ให้ภาพคม

---

## 5. ค่าที่ปรับได้ (`Core/Src/main.c`, ส่วน `USER CODE PD`)

| define | ค่าเริ่มต้น | ความหมาย |
|---|---|---|
| `UART_BAUD` | 2000000 | ความเร็ว UART (Tera Term/Python ต้องตรงกัน) ถ้า Tera Term ไม่รับ ใช้ 921600 |
| `DET_SCALE` | 1 | 1 = หา tag บน 320×240 (ไกลกว่า), 2 = 160×120 (เร็วกว่า) |
| `CONFIRM_FRAMES` | 2 | ต้องเห็น tag เดิมติดกันกี่เฟรมถึงเปลี่ยนคำสั่ง |
| `TAG_LOST_MS` | 1000 | ไม่เห็น tag นานเท่าไร → STOP |
| `OUTPUT_DELAY_MS` | 500 | delay ระหว่างบรรทัดผลใน Tera Term (ปรับตอนรันด้วย `+`/`-`) |
| `VIDEO_SCALE` | 2 | ขนาดภาพที่ส่งให้ Python: 1 = 320×240, 2 = 160×120 |
| `BOOT_DIAG` | 0 | 1 = พิมพ์ผลทดสอบ SCCB / I2C scan / สัญญาณ sync ตอนเปิดเครื่อง |

ค่าของตัวอ่าน tag (`at_default_config` ใน `Core/Src/apriltag_lite.c`): `min_side` 12 px, `min_contrast` 20, `max_hamming` 2

---

## 6. แก้ปัญหา

| อาการ | สาเหตุที่พบบ่อย | วิธีแก้ |
|---|---|---|
| `camera: FAIL - no answer on SCCB` | RESET/PWDN ลอย, ไม่มี pull-up, สาย SIOC/SIOD สลับ/หลวม, ไม่มี XCLK | ตรวจ RESET→3V3, PWDN→GND, ใส่ pull-up 4.2 kΩ, ตั้ง `BOOT_DIAG 1` ดูผลทดสอบ |
| Tera Term เป็นตัวอักษรขยะ | baud ไม่ตรง | ตั้ง Speed = `2000000` (หรือค่า `UART_BAUD`) |
| ไม่มีบรรทัดผลเลย | firmware ไม่ได้ flash / COM ผิด | กด Run ▶ ใน CubeIDE, เลือกพอร์ต STLink, กด RESET |
| `fps` เป็น 0 | VSYNC/HREF/PCLK หลวมหรือมี glitch | ตรวจสาย, เพิ่มสาย GND, แยกสาย VSYNC, ใส่ C 100–470 pF ที่ VSYNC |
| เห็นภาพแต่ไม่เจอ tag | ไม่ใช่ tag36h11, tag เล็กเกิน, เบลอ, แสงสะท้อนจอมือถือ | ใช้รูปใน `tools/tags/`, ถือใกล้ขึ้น, หมุนโฟกัสเลนส์, ลดความสว่างจอ |
| Python: `cannot open COMx` | Tera Term ยังเปิดพอร์ตอยู่ | ปิด Tera Term ก่อน |
| Python: `checksum mismatch` บ่อย | UART ส่งไม่ทัน | ลด `UART_BAUD` เป็น 921600 และรัน `--baud 921600` |

---

## 7. โครงสร้างไฟล์

```
Core/Src/main.c             ลูปหลัก: รับภาพ → หา tag → ตัดสินคำสั่ง → พิมพ์ผล (ส่วน USER CODE)
Core/Src/apriltag_lite.c    ตัวอ่าน AprilTag tag36h11 สำหรับ MCU (ไม่ใช้ malloc)
Core/Src/tag36h11_codes.c   รหัส tag36h11 ทั้ง 587 ตัว
Core/Src/ov7670.c           ตั้งค่ากล้องผ่าน SCCB (QVGA, YUV)
Core/Src/dcmi.c, dma.c      รับภาพจากกล้อง (CubeMX)
Core/Src/i2c.c, tim.c       SCCB (I2C2) และ XCLK (TIM2) (CubeMX)
tools/ov7670_viewer.py      ดูภาพจากกล้อง (แสดงผลอย่างเดียว)
tools/tags/                 รูป tag36h11 ID 1–4 สำหรับทดสอบ
```

## 8. หลักการทำงานโดยย่อ

1. **รับภาพ** — DCMI + DMA เก็บภาพ YUYV 320×240 ลง `frame_buf`, แยกเฉพาะค่า Y (ความสว่าง) เป็นภาพขาวดำ
2. **หา tag** (`at_detect`) — threshold แบบปรับตามแสงในแต่ละบริเวณ → ไล่ขอบก้อนดำ → หาสี่เหลี่ยม (มุมละเอียดระดับ sub-pixel) → homography → อ่านค่า 8×8 ช่อง → เทียบรหัส 587 ตัวทั้ง 4 ทิศ ยอมผิดได้ ≤ 2 bit
3. **ตัดสินคำสั่ง** — ใช้ tag ที่ใหญ่ที่สุด (ใกล้ที่สุด), ต้องเห็นติดกัน `CONFIRM_FRAMES` เฟรม, หายเกิน `TAG_LOST_MS` → STOP
4. **ส่งผล** — ทุก `OUTPUT_DELAY_MS` พิมพ์ 1 บรรทัดไป Tera Term; ถ้า Python ขอ (`v`) จะส่งภาพ + ตำแหน่ง tag (`@TAG`) + คำสั่ง (`@CMD`) ด้วย
