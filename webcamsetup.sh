cd ..
scp -r Webcam pi@wpilibpi.local:/home/pi/
cd Webcam
python find_physical_port.py