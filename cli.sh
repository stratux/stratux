# Script for getting a cli interface into docker

my_uid=${MY_UID:-$(id -u)}
my_gid=${MY_GID:-$(id -g)}

echo "'cd data' to find the local files"
MY_UID="${my_uid}" MY_GID="${my_gid}" docker compose run --rm cli
