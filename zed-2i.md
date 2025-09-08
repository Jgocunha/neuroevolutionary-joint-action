1. v4l2-ctl --list-devices
ZED 2i: ZED 2i (usb-0000:07:00.4-2):
	/dev/video4
	/dev/video5
	/dev/media2

USB2.0 HD UVC WebCam: USB2.0 HD (usb-0000:07:00.4-3):
	/dev/video0
	/dev/video1
	/dev/video2
	/dev/video3
	/dev/media0
	/dev/media1

python3 zed_preview.py --dev /dev/video4 --size 2560x720   # likely SxS
# or for mono:
python3 zed_preview.py --dev /dev/video5 --size 1920x1080

python3 zed_record.py --dev /dev/video4 --size 2560x720 --fps 60 --split
