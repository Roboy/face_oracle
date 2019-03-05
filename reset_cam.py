import sys, os

#resetting camera
try:
    USBDEVFS_RESET= 21780
    f = open("/dev/bus/usb/002/005", 'w', os.O_WRONLY)
    fcntl.ioctl(f, USBDEVFS_RESET, 0)
except Exception, msg:
    print "failed to reset camera: ", msg
