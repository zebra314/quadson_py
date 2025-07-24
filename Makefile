ssh:

sim:
	$(MAKE) sim-build
	$(MAKE) sim-run

sim-build:
	
sim-run:
	xhost +local:root
	docker run -it --rm \
		--mount type=bind,source=$(CURDIR)/sim,target=/root/sim \
		--env="DISPLAY" \
		-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
		-e XDG_RUNTIME_DIR=/tmp \
		-e QT_X11_NO_MITSHM=1 \
		quadson-sim:latest
	xhost -local:root

can:
	$(MAKE) can-build
	$(MAKE) can-run

can-build:
	docker build -f env/Dockerfile.can \
		-t quadson-py-can:latest .

can-run:
	docker run --rm -it \
		--privileged \
		--network=host \
		--cap-add=NET_ADMIN \
		--cap-add=SYS_MODULE \
		-v /lib/modules:/lib/modules:ro \
		--mount type=bind,source=$(CURDIR)/src/real,target=/root/quadson_py/src/real \
		--mount type=bind,source=$(CURDIR)/src/common,target=/root/quadson_py/src/common \
		--mount type=bind,source=$(CURDIR)/tests/real,target=/root/quadson_py/tests/real \
		quadson-py-can:latest zsh
