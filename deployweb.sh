
#!/bin/bash
# Copy all contents of stratux/web to /opt/stratux/www
cp -a "$(dirname "$0")/web/." /opt/stratux/www/

echo "All files copied to /opt/stratux/web folder!"
