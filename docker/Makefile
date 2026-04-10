.PHONY: build build-no-cache run-demo run-interactive run-mac-interactive shell

# Platform detection: Mac needs amd64 emulation and Xpra-based visualization
# instead of X11 socket sharing.
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
  PLATFORM_FLAG = --platform linux/amd64
else
  PLATFORM_FLAG =
endif

# GPU support toggle: make run-demo GPU=false to disable
GPU ?= true
ifeq ($(GPU),true)
GPU_FLAGS = --gpus all --env="NVIDIA_DRIVER_CAPABILITIES=all"
else
GPU_FLAGS =
endif

DOCKER_FLAGS = xhost + && docker run -it \
	$(GPU_FLAGS) \
	--privileged \
	-e DISPLAY=$(DISPLAY) \
	--network=host \
	--env="QT_X11_NO_MITSHM=1" \
	--rm

# ---- Build ----
# BUILD_JOBS controls build parallelism. Default 2 works on machines with ~8 GB RAM.
# Increase it (e.g. BUILD_JOBS=4) if you have more RAM — each parallel compiler
# process can consume up to 4 GB on template-heavy files like gazebo_ros_camera.
# Decrease to 1 if you hit "cannot allocate memory" or "killed cc1plus" errors.
BUILD_JOBS ?= 2

# SANDO_TAG selects which GitHub tag/branch the Dockerfile clones. Defaults to
# the Dockerfile's own default (`main`). Override with `make build SANDO_TAG=v1.0.0`
# once a release tag is cut. Passing an empty value preserves the Dockerfile default.
SANDO_TAG ?= v0.0.2
ifneq ($(SANDO_TAG),)
TAG_ARG = --build-arg SANDO_TAG=$(SANDO_TAG)
else
TAG_ARG =
endif

build:
	docker build $(PLATFORM_FLAG) -t sando \
		--build-arg BUILD_JOBS=$(BUILD_JOBS) $(TAG_ARG) .

build-no-cache:
	docker build $(PLATFORM_FLAG) --no-cache -t sando \
		--build-arg BUILD_JOBS=$(BUILD_JOBS) $(TAG_ARG) .

# ---- Demo mode: pre-configured scenarios with automatic goal ----
# Usage: make run-demo SCENARIO=static_easy
# Scenarios: static_easy, static_medium, static_hard,
#            dynamic_easy, dynamic_medium, dynamic_hard,
#            unknown_dynamic_easy, unknown_dynamic_medium, unknown_dynamic_hard

SCENARIO ?= static_easy

run-demo:
	$(DOCKER_FLAGS) -e MODE=demo -e SCENARIO=$(SCENARIO) sando

# Convenience aliases
run-static-easy:
	$(MAKE) run-demo SCENARIO=static_easy
run-static-medium:
	$(MAKE) run-demo SCENARIO=static_medium
run-static-hard:
	$(MAKE) run-demo SCENARIO=static_hard
run-dynamic-easy:
	$(MAKE) run-demo SCENARIO=dynamic_easy
run-dynamic-medium:
	$(MAKE) run-demo SCENARIO=dynamic_medium
run-dynamic-hard:
	$(MAKE) run-demo SCENARIO=dynamic_hard
run-unknown-easy:
	$(MAKE) run-demo SCENARIO=unknown_dynamic_easy
run-unknown-medium:
	$(MAKE) run-demo SCENARIO=unknown_dynamic_medium
run-unknown-hard:
	$(MAKE) run-demo SCENARIO=unknown_dynamic_hard

# ---- Interactive mode: click goals in RViz ----
# Usage: make run-interactive
#        make run-interactive NUM_OBSTACLES=100 DYNAMIC_RATIO=0.8

NUM_OBSTACLES ?= 50
DYNAMIC_RATIO ?= 0.65

run-interactive:
	$(DOCKER_FLAGS) -e MODE=interactive -e NUM_OBSTACLES=$(NUM_OBSTACLES) -e DYNAMIC_RATIO=$(DYNAMIC_RATIO) sando

# ---- Mac interactive mode: browser-based RViz via Xpra ----
# Usage: make run-mac-interactive
#        make run-mac-interactive NUM_OBSTACLES=100 DYNAMIC_RATIO=0.8
# Then open http://localhost:8080 in your browser.
# Uses Xpra instead of X11 passthrough (which doesn't work on macOS Docker Desktop).
run-mac-interactive:
	docker run $(PLATFORM_FLAG) -it \
		--privileged \
		--env="QT_X11_NO_MITSHM=1" \
		-e USE_XPRA=true \
		-e MODE=interactive \
		-e NUM_OBSTACLES=$(NUM_OBSTACLES) \
		-e DYNAMIC_RATIO=$(DYNAMIC_RATIO) \
		-p 8080:8080 \
		--rm sando

# ---- Debug shell ----
shell:
	$(DOCKER_FLAGS) --entrypoint /bin/bash sando
