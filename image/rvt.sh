#!/bin/bash

#chronyd -n
 
#./mk_europe_edition64.sh prod bluetooth-le-devices
chronyd -q
./mk_europe_edition64.sh dev bluetooth-le-devices

