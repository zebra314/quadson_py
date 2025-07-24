# quadson_py

## Can bus control

## Simulation

## Build

if

```sh
modprobe: FATAL: Module can not found in directory /lib/modules/6.12.28-1-lts
```

你可以先在實體機（或 VM）上用以下指令檢查 host kernel 是否支援 CAN：

```sh
ls /lib/modules/$(uname -r)/kernel/net/can
```

正常會看到：

```sh
can.ko  can-dev.ko  can-raw.ko
```

```sh
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
```

### Docker

```sh
make can-build
```

```sh
make sim-build
```

### Python virtual environment

Activate your virtual environment and run:

```sh
pip install -r env/requirements-can.txt
```

```sh
pip install -r env/requirements-sim.txt
```

Then

```sh
pip install -e .
```
