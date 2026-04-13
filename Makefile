# ==============================================================================
# Configuration — edit these to match your setup
# ==============================================================================
FQBN       := esp8266:esp8266:nodemcu
PORT       := /dev/ttyUSB0
BAUD       := 115200
SKETCH_DIR := $(CURDIR)

# Add or remove libraries your project needs
LIBS := \
	"DHT sensor library" \
	"MH-Z19" \
	"PubSubClient" \
	"EspSoftwareSerial"

# ==============================================================================
# Targets
# ==============================================================================
.PHONY: help libs compile upload monitor flash

help:
	@echo ""
	@echo "  make libs     — install required Arduino libraries"
	@echo "  make compile  — compile the sketch"
	@echo "  make upload   — upload to board (default port /dev/ttyUSB0)"
	@echo "  make monitor  — open serial monitor"
	@echo "  make flash    — compile + upload in one step"
	@echo ""
	@echo "  PORT=$(PORT)  FQBN=$(FQBN)"
	@echo ""

libs:
	arduino-cli lib update-index
	@for lib in $(LIBS); do \
		echo "Installing $$lib..."; \
		arduino-cli lib install $$lib; \
	done

compile:
	arduino-cli compile --fqbn $(FQBN) $(SKETCH_DIR)

upload:
	@if [ -z "$(PORT)" ]; then \
		echo "Error: No board detected. Plug in your device or set PORT manually: make upload PORT=/dev/ttyUSB0"; \
		exit 1; \
	fi
	arduino-cli upload --fqbn $(FQBN) --port $(PORT) $(SKETCH_DIR)

monitor:
	@if [ -z "$(PORT)" ]; then \
		echo "Error: No board detected. Set PORT manually: make monitor PORT=/dev/ttyUSB0"; \
		exit 1; \
	fi
	arduino-cli monitor --port $(PORT) --config baudrate=$(BAUD)

flash: compile upload
