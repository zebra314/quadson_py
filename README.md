# quadson_py

## CAN bus control

For Docker users, use the following command to build and run the environment.

```sh
make can
```

For python venv users, activate your new virtual environment and run:

```sh
pip install -r env/requirements-can.txt
pip install -e .
```

When launching the program, if you see the error messages from terminal like:

```sh
modprobe: FATAL: Module can not found in directory /lib/modules/6.12.28-1-lts
```

This means that the host kernel does not support CAN bus. First, check if the host kernel supports CAN bus by running:

```sh
ls /lib/modules/$(uname -r)/kernel/net/can
```

You should see something like:

```sh
can.ko  can-dev.ko  can-raw.ko
```

Then load the module manually by running:

```sh
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
```

and restart the program, the problem should be solved.

## Simulation
