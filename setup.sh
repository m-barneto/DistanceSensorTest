#!/bin/bash

apt-get install python3-dev
python -m venv venv
source ./venv/bin/activate
pip install -r requirements.txt
