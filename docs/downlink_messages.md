# LoRa devices

## Downlinks to devices

It is possible to set uplink interval via downlink message.
Default state is short ~40 sec messages

Downlink can be sent via MQTT topic:

    v3/via-smart-things@ttn/devices/DEV-EUI/down/push

DEV-EUI should be replaced with specific devices eui.

30 min

```json
{
   "downlinks": [{
     "f_port": 1,
     "frm_payload": "UQAe",
     "priority": "NORMAL"
   }]
 }
```

10 min

```json
{
   "downlinks": [{
     "f_port": 1,
     "frm_payload": "UQAK",
     "priority": "NORMAL"
   }]
 }
```

5 min

```json
{
   "downlinks": [{
     "f_port": 1,
     "frm_payload": "UQAF",
     "priority": "NORMAL"
   }]
 }
```

reset Node

```json
{
   "downlinks": [{
     "f_port": 1,
     "frm_payload": "EA==",
     "priority": "NORMAL"
   }]
 }
```