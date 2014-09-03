Autostart ardupilot
-------------------

```bash
cp apm4 apm4-startup.sh /etc/init.d
update-rc.d apm4 defaults
```

Finally, place a binary called "ArduCopter.elf" at `/root`.

