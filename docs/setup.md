# Virtual Environment Setup

### 1. Create the Virtual Environment
> python -m venv .venv

### 2. Activate environment 
> .venv\Scripts\activate **(Windows)** <br>
> .venv/bin/activate **(MacOS/Linux)** <br>
> source .venv/bin/activate **(RPi)**

### 3. Upgrade Package Manager
> python -m pip install --upgrade pip

### 4. Install Requirements
> pip install -r requirements.txt

# Install dependecies (RPi)
> sudo apt update <br>
> sudo apt install -y swig python3-dev build-essential <br>
> sudo apt install liblgpio-dev <br>
> pip install rpi-lgpio