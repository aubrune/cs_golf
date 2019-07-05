# Activate autostart on this machine

3 services are needed:
* The roscore (service named `cs_golf_roscore`)
* The REST API (service named `cs_golf_api`)
* The main interaction controller (service named `cs_golf_controller`)

```
sudo cp cs_golf*.service /lib/systemd/system/
sudo cp power /etc/acpi/events/power
sudo systemctl daemon-reload
sudo systemctl enable cs_golf_roscore.service
sudo systemctl enable cs_golf_api.service
sudo systemctl enable cs_golf_controller.service
```

## Manage services at runtime

```
sudo service cs_golf_controller status
sudo service cs_golf_api status
sudo service cs_golf_roscore status

sudo journalctl -u cs_golf_controller -f -a
```
