.PHONY: help setup run clean

PYTHON ?= python
HOST ?= 0.0.0.0
PORT ?= 8340

help:
	@$(PYTHON) scripts/dev.py help

setup:
	@$(PYTHON) scripts/dev.py setup

run:
	@$(PYTHON) scripts/dev.py run --host "$(HOST)" --port "$(PORT)"

clean:
	@$(PYTHON) scripts/dev.py clean
