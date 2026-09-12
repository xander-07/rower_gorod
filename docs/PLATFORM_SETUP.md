# Платформа Raspberry Pi 4 — текущее состояние

Дата диагностики: 2026-09-12.

## Актуальная конфигурация

Подтверждено на реальном роботе:

- Raspberry Pi 4, 4 GB RAM;
- Debian GNU/Linux 12 (Bookworm), arm64/aarch64;
- Python 3.11.2;
- Waveshare `ugv_rpi` находится в `~/ugv_rpi`;
- штатное окружение Waveshare: `~/ugv_rpi/ugv-env`, Python 3.11.2;
- свободно/доступно около 3.2 GiB RAM в момент диагностики;
- swap: 512 MiB;
- Docker изначально не установлен;
- ОС не меняем, так как именно эта конфигурация нужна для рабочего Waveshare-стека.

## Waveshare UGV02

Интерфейс нижнего ESP32:

```text
/dev/serial0 -> /dev/ttyAMA0
115200 baud
newline-delimited JSON
```

Подтверждено:

- `T=1001` telemetry;
- `T=1` left/right velocity control;
- `T=13` ROS-style X/Z velocity control;
- энкодеры и одометрия;
- напряжение батареи (`v / 100` = V).

Для будущего `rower_base_bridge` предпочтителен `T=1`, потому что в актуальной прошивке Waveshare его обработчик обновляет heartbeat. Преобразование ROS `linear.x` / `angular.z` в `L/R` будет выполняться в нашем узле.

## Built-in IMU

После переключения в `moduleType=0` и ручных наклонов/поворотов получено:

```text
SUMMARY: T1001=161 T1005=0 malformed=0
RANGES:
  gx: 0 .. 0
  gy: 0 .. 0
  gz: 0 .. 0
  ax: 0 .. 0
  ay: 0 .. 0
  az: 0 .. 0
  mx: 0 .. 0
  my: 0 .. 0
  mz: 0 .. 0
```

Поэтому встроенный IMU не используется в первой версии навигации. Это не блокирует SLAM/Nav2: используем wheel odometry + STL-19P.

## LDROBOT STL-19P

Подтверждено:

- CP2102 `10c4:ea60`, serial `0001`;
- 230400 baud;
- 47-byte LD19/STL-19P packets;
- 12 points/frame;
- 50/50 CRC-valid frames;
- 592/600 valid points;
- ~9.91–9.93 Hz spin rate;
- полный проход 360 -> 0 градусов.

Постоянное имя:

```text
/dev/rower_lidar -> /dev/ttyUSB0
```

Установка udev rule:

```bash
cd ~/rower_gorod
chmod +x scripts/install_udev_rules.sh
./scripts/install_udev_rules.sh
```

## Конфликт со штатным `ugv_rpi/app.py`

`ugv_rpi/app.py` автоматически открывает первый `/dev/ttyUSB*` и конфликтует с STL-19P.

На роботе приложение запускается user service:

```text
~/.config/systemd/user/ugv-app.service
```

В unit установлен `Restart=always`. Во время автономной разработки сервис должен оставаться отключенным:

```bash
systemctl --user disable ugv-app.service
systemctl --user stop ugv-app.service
```

Проверка:

```bash
pgrep -af 'ugv_rpi/app.py' || true
sudo fuser -v /dev/ttyUSB0 || true
```

`ugv-jupyter.service` можно оставить запущенным, пока он не захватывает нужные serial devices.

## ROS 2

ROS 2 Jazzy не устанавливаем прямо в Debian 12 host. Вместо смены ОС используем официальный Ubuntu 24.04 arm64 ROS 2 container поверх Docker.

Файлы проекта:

```text
docker/Dockerfile
scripts/setup_docker_bookworm.sh
scripts/build_ros2_docker.sh
scripts/run_ros2_docker.sh
```

Подробно: `docs/ROS2_SETUP.md`.

## Целевая схема

```text
Debian 12 / Raspberry Pi 4
├── Waveshare vendor stack: ~/ugv_rpi
├── project: ~/rower_gorod
└── Docker
    └── ROS 2 Jazzy / Ubuntu 24.04 arm64
        ├── rower_base_bridge
        ├── rower_lidar
        ├── rower_description
        ├── rower_bringup
        ├── SLAM Toolbox
        └── Nav2
```

RViz планируется запускать на отдельном ноутбуке в той же сети, чтобы не нагружать Raspberry Pi 4 графикой.
