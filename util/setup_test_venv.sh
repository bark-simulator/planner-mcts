#!/bin/bash
mkdir -p bark_mcts/python_wrapper/venv
virtualenv --system-site-packages -p python3 bark_mcts/python_wrapper/venv
source bark_mcts/python_wrapper/venv/bin/activate && pip install -r util/requirements.txt
