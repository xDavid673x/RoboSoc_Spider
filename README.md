# Hexapod Web Control Quick Start

## Webots Simulator

The standalone six-leg Webots R2025a project, keyboard controls, slope worlds,
provisional dynamics, and verification instructions are documented in
[`webots/README.md`](webots/README.md).

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
[init_db.php](https://web.cs.manchester.ac.uk/c59506kl/hexapod_robot/web_hosting/init_db.php?api_token=97af9d5e3b1287eb4b1f1266820f9dbaaf49f57c137e9c30ac339952217e4582)

### Controller URL
Open:
[web controller](https://web.cs.manchester.ac.uk/c59506kl/hexapod_robot/web_hosting/)

### Pi (or machine with servo connected) receiver
```bash
python3 web_controller/pi_remote_client.py \
  --endpoint "https://web.cs.manchester.ac.uk/c59506kl/hexapod_robot/web_hosting/get_command.php" \
  --token "97af9d5e3b1287eb4b1f1266820f9dbaaf49f57c137e9c30ac339952217e4582" \
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

## Logging Modes

- Quiet important-only logs are the default.
- Add `--print-latency` to `pi_remote_client.py` if you want periodic `total_latency_ms=...` lines.
- Add `--verbose-stream` to `pi_control_server.py` if you want continuous detailed state stream logs.

## Raspberry Pi Auto-Start (systemd)

Create env file (store token safely):

```bash
sudo tee /etc/hexapod_remote.env >/dev/null <<'EOF'
ENDPOINT=https://web.cs.manchester.ac.uk/c59506kl/hexapod_robot/web_hosting/get_command.php
API_TOKEN=97af9d5e3b1287eb4b1f1266820f9dbaaf49f57c137e9c30ac339952217e4582
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
