#!/bin/sh

export PROBCOG_HOME=$HOME/probcog
export JAVA=/usr/bin/java
export JAVA_OPTS_64="-Xmx64m"
export JAVA_OPTS_128="-Xmx128m"
export JAVA_OPTS_256="-Xmx256m"
export JAVA_OPTS_512="-Xmx512m"
export JAVA_OPTS_1024="-Xmx1024m"

PROC_CONFIG=$PROBCOG_HOME/config/procman/proc.config
export BIN_PATH=$PROBCOG_HOME/bin

# Networking
# XXX Not necessary for now

# Start the software update daemon (in java)
# XXX Currently not implemented

# Start the tunupd for the mesh network
# XXX Currently not implemented

# XXX Why is this not always true?
sudo swapoff -a

echo "Setting route tables on lo, wlan0 for LCM"
sudo ifconfig lo multicast
sudo route add -net 239.255.76.0/24 dev lo
sudo ifconfig wlan0 multicast
sudo route add -net 239.255.11.0/24 dev wlan0

# Step 1: start ProcManDaemon
export DAEMON_CMD="$JAVA $JAVA_OPTS_64 april.procman.ProcManDaemon"

# Step 2: start ProcMan with correct config file:
export CONTROLLER_CMD="$JAVA $JAVA_OPTS_64 april.procman.ProcMan -c $PROC_CONFIG"

$DAEMON_CMD &
$CONTROLLER_CMD
