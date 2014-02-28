#!/bin/sh

export PROBCOG_HOME=/home/april/probcog
export APRIL_HOME=/home/april/april
export ORC_HOME=/home/april/uorc
export SOAR_HOME=/home/april/Soar/out
export JAVA=/usr/bin/java
export JAVA_OPTS_64="-ea -server -Xmx64m"
export JAVA_OPTS_128="-ea -server -Xmx128m"
export JAVA_OPTS_256="-ea -server -Xmx256m"
export JAVA_OPTS_512="-ea -server -Xmx512m"
export JAVA_OPTS_1024="-ea -server -Xmx1024m"

export CLASSPATH=$APRIL_HOME/java/april.jar:$ORC_HOME/user/java/orc.jar:/usr/local/share/java/lcm.jar:$PROBCOG_HOME/java/probcog.jar:$SOAR_HOME/java/sml.jar:$SOAR_HOME/SoarJavaDebugger.jar
export LD_LIBRARY_PATH=$APRIL_HOME/lib:/usr/local/lib:/usr/local/lib64:$SOAR_HOME
export CPATH=$SOAR_HOME/include # XXX Necessary?

PROC_CONFIG=$PROBCOG_HOME/config/procman/proc.config
export BIN_PATH=$PROBCOG_HOME/bin

# Networking
# XXX Not necessary for now

# Start the software update daemon (in java)
# XXX Currently not implemented

# Start the tunupd for the mesh network
# XXX Currently not implemented

# XXX Why is this not always true?
swapoff -a

echo "Setting route tables on lo, wlan0 for LCM"
ifconfig lo multicast
route add -net 239.255.76.0/24 dev lo
ifconfig wlan0 multicast
route add -net 239.255.11.0/24 dev wlan0

# Step 1: start ProcManDaemon
export DAEMON_CMD="$JAVA $JAVA_OPTS_64 april.procman.ProcManDaemon"

# Step 2: start ProcMan with correct config file:
export CONTROLLER_CMD="$JAVA $JAVA_OPTS_64 april.procman.ProcMan -c $PROC_CONFIG"

$DAEMON_CMD &
$CONTROLLER_CMD
