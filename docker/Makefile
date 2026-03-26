.PHONY: build build-no-cache run-demo run-interactive shell

# GPU support toggle: make run-demo GPU=false to disable
GPU ?= true
ifeq ($(GPU),true)
GPU_FLAGS = --gpus all --env="NVIDIA_DRIVER_CAPABILITIES=all"
else
GPU_FLAGS =
endif

DOCKER_RUN = xhost + && docker run -it \
	$(GPU_FLAGS) \
	--privileged \
	-e DISPLAY=$(DISPLAY) \
	--network=host \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--rm sando

# ---- Build ----

build:
	docker build -t sando .

build-no-cache:
	docker build --no-cache -t sando .

# ---- Demo mode: pre-configured scenarios with automatic goal ----
# Usage: make run-demo SCENARIO=static_easy
# Scenarios: static_easy, static_medium, static_hard,
#            dynamic_easy, dynamic_medium, dynamic_hard,
#            unknown_dynamic_easy, unknown_dynamic_medium, unknown_dynamic_hard

SCENARIO ?= static_easy

run-demo:
	$(DOCKER_RUN) -e MODE=demo -e SCENARIO=$(SCENARIO)

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
	$(DOCKER_RUN) -e MODE=interactive -e NUM_OBSTACLES=$(NUM_OBSTACLES) -e DYNAMIC_RATIO=$(DYNAMIC_RATIO)

# ---- Debug shell ----
shell:
	xhost + && docker run -it \
		$(GPU_FLAGS) \
		--privileged \
		-e DISPLAY=$(DISPLAY) \
		--network=host \
		--env="DISPLAY" \
		--env="QT_X11_NO_MITSHM=1" \
		--rm --entrypoint /bin/bash sando
