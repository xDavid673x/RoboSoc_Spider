# Hexapod Web Control Quick Start

## Webots Spider Simulator

The standalone six-leg Webots R2025a simulator runs without a Servo 2040 board.
It provides CAD-derived leg geometry, flat and sloped validation worlds, and
keyboard control. The detailed model and calibration notes are in
[webots/README.md](webots/README.md).

### Demonstration

![Webots spider tripod gait demonstration](webots/assets/demos/spider-tripod-gait.gif)

*Top-down Webots demonstration of the spider's tripod gait moving across uneven
terrain.*

### Start on macOS

Install Webots R2025a, then run these commands from the repository root:

```bash
WEBOTS=/Applications/Webots.app/Contents/MacOS/webots
"$WEBOTS" webots/worlds/flat.wbt
```

Choose another world by replacing the filename:

| World | Purpose |
| --- | --- |
| `flat.wbt` | Flat-ground motion and physics smoke test |
| `slope_10.wbt`, `slope_20.wbt`, `slope_30.wbt` | 10°, 20°, and 30° slope checks |
| `uneven_terrain_spider.wbt` | R2025a uneven-terrain integration scene |
| `realistic_village_spider.wbt` | Realistic Village visual/integration scene |

Click the 3D view to give it keyboard focus:

| Key | Action |
| --- | --- |
| `W` / `S` | Forward / backward |
| `A` / `D` | Parallel left / right |
| `J` / `K` | In-place left / right turn |
| `Space` | Stop and hold the initial stance |
| `R` | Restore the world pose, joints, gait phase, and physics |

`W`/`S` can be combined with `A`/`D` for diagonal translation. `J`/`K` take
precedence when held with a translation key.

### Validate the simulator

Run the generated-geometry check and the full Python/Webots test suite from the
repository root:

```bash
python3 tools/cad_sync.py check-generated
python3 -m pytest -q
```

The flat and slope worlds are the canonical headless smoke-test targets.
The uneven-terrain and village scenes use Webots R2025a external resources and
are intended for interactive integration checks.


Shared UI source (used by both internet and LAN modes):
- `shared_ui/index.html`
- `shared_ui/styles.css`

## 1. Detect Servo Board Serial Port

### macOS
```bash
ls /dev/cu.usbmodem*
```

### Raspberry Pi / Linux
```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

Use the detected device path in the commands below.

## 2. Internet Web Control (University Hosting)

### Health / DB init check
Open:
[init_db.php](https://web.cs.manchester.ac.uk/c59506kl/hexapod_robot/web_hosting/init_db.php)

### Controller URL
Open:
[web controller](https://web.cs.manchester.ac.uk/c59506kl/hexapod_robot/web_hosting/)

### Pi (or machine with servo connected) receiver

Set the token in the environment; do not put the secret in this command or in
the repository:

```bash
export HEXAPOD_API_TOKEN='replace-with-your-token'
```

```bash
python3 web_controller/pi_remote_client.py \
  --endpoint "https://web.cs.manchester.ac.uk/c59506kl/hexapod_robot/web_hosting/get_command.php" \
  --token "$HEXAPOD_API_TOKEN" \
  --poll-hz 40 \
  --http-timeout-s 0.4 \
  --stale-timeout-s 1.5 \
  --serial-port /dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6617c93e39c662b-if00 \
  --baud 115200 \
  --serial-required
```

Note:
- On macOS, serial port may look like `/dev/cu.usbmodem2101`.
- On Raspberry Pi, serial port is usually `/dev/ttyACM0` or `/dev/ttyUSB0`.

## 3. Local Web Control (LAN)

### Get local IP

#### macOS
```bash
ipconfig getifaddr en0
```

#### Raspberry Pi / Linux
```bash
hostname -I
```

### Run local receiver server
```bash
python3 web_controller/pi_control_server.py \
  --serial-port /dev/ttyACM0 \
  --baud 115200 \
  --serial-required
```

python3 web_controller/pi_control_server.py \
  --serial-port /dev/cu.usbmodem2101 \
  --baud 115200 \
  --serial-required

Then open:

`http://<local-ip>:8080`

## Security

- Do not commit real API tokens/passwords into public repos.
- Keep `web_hosting/config.php` secret in production.
- Export `HEXAPOD_API_TOKEN` in your shell before using the remote receiver.
- The credentials currently present in `web_hosting/config.php` must be revoked
  and rotated; keep that deployment-only file out of public history.

## Logging Modes

- Quiet important-only logs are the default.
- Add `--print-latency` to `pi_remote_client.py` if you want periodic `total_latency_ms=...` lines.
- Add `--verbose-stream` to `pi_control_server.py` if you want continuous detailed state stream logs.

## Raspberry Pi Auto-Start (systemd)

Create env file (store token safely):

```bash
sudo tee /etc/hexapod_remote.env >/dev/null <<'EOF'
ENDPOINT=https://web.cs.manchester.ac.uk/c59506kl/hexapod_robot/web_hosting/get_command.php
API_TOKEN=replace-with-your-token
SERIAL_PORT=/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6617c93e39c662b-if00
EOF
sudo chmod 600 /etc/hexapod_remote.env
```

Create service:

```bash
sudo tee /etc/systemd/system/hexapod-remote.service >/dev/null <<'EOF'
[Unit]
Description=Hexapod Internet Receiver
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=spyder
SupplementaryGroups=dialout
WorkingDirectory=/home/spyder/Desktop/RoboSoc_Spider
EnvironmentFile=/etc/hexapod_remote.env
ExecStart=/usr/bin/python3 /home/spyder/Desktop/RoboSoc_Spider/web_controller/pi_remote_client.py --endpoint ${ENDPOINT} --token ${API_TOKEN} --poll-hz 40 --http-timeout-s 0.4 --stale-timeout-s 1.5 --serial-port ${SERIAL_PORT} --baud 115200 --serial-required
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF
```

Enable and start now:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now hexapod-remote.service
```

Check status/logs:

```bash
systemctl is-enabled hexapod-remote.service
systemctl status hexapod-remote.service
journalctl -u hexapod-remote.service -f
```

Stop once:

```bash
sudo systemctl stop hexapod-remote.service
```

Disable auto-start on boot:

```bash
sudo systemctl disable hexapod-remote.service
```

## Raspberry Pi Auto-Start (LAN Local Server)

Create env file:

```bash
sudo tee /etc/hexapod_lan.env >/dev/null <<'EOF'
SERIAL_PORT=/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6617c93e39c662b-if00
HOST=0.0.0.0
PORT=8080
EOF
sudo chmod 600 /etc/hexapod_lan.env
```

Create service:

```bash
sudo tee /etc/systemd/system/hexapod-lan.service >/dev/null <<'EOF'
[Unit]
Description=Hexapod LAN Web Controller
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=spyder
SupplementaryGroups=dialout
WorkingDirectory=/home/spyder/Desktop/RoboSoc_Spider
EnvironmentFile=/etc/hexapod_lan.env
ExecStart=/usr/bin/python3 /home/spyder/Desktop/RoboSoc_Spider/web_controller/pi_control_server.py --host ${HOST} --port ${PORT} --serial-port ${SERIAL_PORT} --baud 115200 --serial-required
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF
```

Enable and start now:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now hexapod-lan.service
```

Check status/logs:

```bash
systemctl is-enabled hexapod-lan.service
systemctl status hexapod-lan.service
journalctl -u hexapod-lan.service -f
```

Stop once:

```bash
sudo systemctl stop hexapod-lan.service
```

Disable auto-start on boot:

```bash
sudo systemctl disable hexapod-lan.service
```
