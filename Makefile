#to build for ARM run:
#  make ARCH=arm
#to build for x86 run:
#  make

PROJECT_NAME=game
OBJS=main.o game_behavior.o frame_grabber.o frame_processing.o

#Append all paths to directories containing ARM shared libraries to this space
#separated list
PATHS_TO_ARM_SHARED_LIBS= \
/home/michael/programming/senior_design/libs_on_ramdisk

#Append all paths to directories containing ARM header files to this space
#separated list
PATHS_TO_ARM_INCLUDES= \
/home/michael/programming/senior_design/opencv/install/include \
/home/michael/programming/senior_design/ncurses-5.9/ramdisk/usr/include


CROSS_CXX=arm-xilinx-linux-gnueabi-g++
NATIVE_CXX=g++

SRC=./src
INC=./include
VPATH=$(SRC):$(INC)

ifeq ($(ARCH),arm)

$(info )
$(info "Building $(PROJECT_NAME) for ARM architecture")
$(info )

CXX=$(CROSS_CXX)
CXXFLAGS=$(foreach directory,$(PATHS_TO_ARM_INCLUDES),-I$(directory))
LIBS= \
$(foreach directory,$(PATHS_TO_ARM_SHARED_LIBS),-L$(directory)) \
-lopencv_contrib \
-lopencv_legacy \
-lopencv_objdetect \
-lopencv_video \
-lopencv_ml \
-lopencv_calib3d \
-lopencv_features2d \
-lopencv_highgui \
-lopencv_flann \
-lopencv_imgproc \
-lopencv_core \
-lrt \
-lpthread \
-lncurses

else 

$(info )
$(info "Building $(PROJECT_NAME) for x86 architecture")
$(info )

CXX=$(NATIVE_CXX)
CXXFLAGS=`pkg-config --cflags opencv`
LIBS=`pkg-config --libs opencv` \
-lncurses

endif

CXXFLAGS+= -I$(INC)
CXXFLAGS+= -fpermissive
CXXFLAGS+= -Wall
LD=$(CXX)

all:$(PROJECT_NAME) 

$(PROJECT_NAME):$(OBJS)
	$(LD) -o $@ $^ $(LIBS)

%.o:%.cpp
	$(CXX) -o $@ $^ -c $(CXXFLAGS)  

clean:
	rm -f $(PROJECT_NAME)
	rm -f $(OBJS)
