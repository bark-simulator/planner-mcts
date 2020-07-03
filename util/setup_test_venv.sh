#!/bin/bash
mkdir -p bark/python_wrapper/venv
virtualenv --system-site-packages -p python3 bark/python_wrapper/venv
source bark/python_wrapper/venv/bin/activate
