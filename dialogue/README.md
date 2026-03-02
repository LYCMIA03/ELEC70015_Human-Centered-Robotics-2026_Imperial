# Installation Guide

```
conda create -n hcr python=3.10
conda activate hcr
pip install -r requirements.txt
```

For microphone usage, please install the following packages: 

```
# For Ubuntu
sudo apt-get install -y libportaudio2 portaudio19-dev

# Verify installation
python -c "import sounddevice as sd; print(sd.query_devices())"
```