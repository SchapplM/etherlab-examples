# Make-File für das ROS-Simulink-Interface

# Lucas Jürgens (BA), lucas.juergens@zubox.de, 2017-02
# Moritz Schappler, schappler@irt.uni-hannover.de
# (c) Institut für Regelungstechnik, Universität Hannover


BUILDDIR=build
ROS_NODE_DIR=ros_rt_core
# ROS_PATH=$OECORE_TARGET_SYSROOT/opt/ros/indigo
ROS_NODE_FILE=SL_func_dummy
LIB_NAME=libros_sl_interface.so

all: roslib

roslib: $(BUILDDIR)/$(roslib.a)
	${CXX} -fPIC -c -DDUMMY $(ROS_NODE_DIR)/$(ROS_NODE_FILE).cpp -o $(BUILDDIR)/$(ROS_NODE_FILE).o -Wall -shared
	${CXX} -shared -fPIC -Wl,-soname,$(LIB_NAME) -o $(BUILDDIR)/$(LIB_NAME) $(BUILDDIR)/$(ROS_NODE_FILE).o -lc
