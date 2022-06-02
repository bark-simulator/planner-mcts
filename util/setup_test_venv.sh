#!/bin/bash
mkdir -p bark_mcts/python_wrapper/venv
virtualenv --system-site-packages -p python3 bark_mcts/python_wrapper/venv
source bark_mcts/python_wrapper/venv/bin/activate && pip install -r util/requirements.txt
pip install torch==1.9.0+cpu torchvision -f https://download.pytorch.org/whl/torch_stable.html