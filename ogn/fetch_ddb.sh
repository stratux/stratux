#!/bin/bash
echo "Fetching ddb.json from ddb.glidernet.org"
wget -qO ddb.json http://ddb.glidernet.org/download/?j=1
if [ "$?" -ne 0 ]; then
	echo "Error downloading the file. Using the existing copy stored in ddb.json.copy"
	cp -f ddb.json.copy ddb.json
fi
exit 0
