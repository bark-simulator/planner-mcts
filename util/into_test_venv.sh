#!/bin/bash
source bark_mcts/python_wrapper/venv/bin/activate
export LD_LIBRARY_PATH = $LD_LIBRARY_PATH:$PWD/bark_mcts/python_wrapper/venv/lib/python3.7/site-packages/torch/lib/ 
